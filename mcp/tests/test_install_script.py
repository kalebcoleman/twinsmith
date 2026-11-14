"""Check that host-only installation preserves an existing ffmpeg executable."""

import os
from pathlib import Path
import subprocess
import sys

import pytest


@pytest.mark.parametrize("existing", ["file", "symlink", "missing", "owned"])
def test_host_install_preserves_unrelated_ffmpeg(tmp_path, existing):
    home = tmp_path / "home"
    bin_dir = home / ".local" / "bin"
    bin_dir.mkdir(parents=True)
    ffmpeg = bin_dir / "ffmpeg"
    original = b"user's existing ffmpeg\n"
    target = tmp_path / "existing executable"
    if existing == "file":
        ffmpeg.write_bytes(original)
    elif existing == "symlink":
        target.write_bytes(original)
        ffmpeg.symlink_to(target)
    elif existing == "owned":
        ffmpeg.write_text("# ffmpeg shim: prefer the static build shipped with imageio-ffmpeg.\nold shim\n")

    fake_bin = tmp_path / "fake-bin"
    fake_bin.mkdir()
    uv = fake_bin / "uv"
    uv.write_text(
        '#!/bin/sh\n'
        'mkdir -p "$HOME/.local/bin"\n'
        'printf "#!/bin/sh\\nexit 0\\n" > "$HOME/.local/bin/twinsmith"\n'
        'chmod +x "$HOME/.local/bin/twinsmith"\n'
        'prev=\n'
        'for arg in "$@"; do\n'
        '  if [ "$prev" = "--project" ]; then\n'
        '    mkdir -p "$arg/.venv/bin"\n'
        '    printf "#!/bin/sh\\nexit 0\\n" > "$arg/.venv/bin/twinsmith"\n'
        '    chmod +x "$arg/.venv/bin/twinsmith"\n'
        '  fi\n'
        '  prev=$arg\n'
        'done\n'
    )
    uv.chmod(0o755)
    env = os.environ | {"HOME": str(home), "PATH": f"{fake_bin}{os.pathsep}{os.environ['PATH']}"}
    script = Path(__file__).resolve().parents[2] / "install.sh"
    result = subprocess.run(["bash", str(script), "--no-sim"], env=env, capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
    if existing in {"file", "symlink"}:
        assert ffmpeg.read_bytes() == original
        assert ffmpeg.is_symlink() == (existing == "symlink")
        assert "preserving existing" in result.stderr
    else:
        assert "ffmpeg shim: prefer the static build" in ffmpeg.read_text()
        assert "old shim" not in ffmpeg.read_text()
        assert os.access(ffmpeg, os.X_OK)



REPO = Path(__file__).resolve().parents[2]


def _fake_isaac(venv: Path, version: str) -> Path:
    """A pip-style environment that env.py reports as Isaac Sim *version* (never executed)."""
    (venv / "bin").mkdir(parents=True)
    python = venv / "bin" / "python"
    python.write_text("#!/bin/sh\nexit 0\n")
    python.chmod(0o755)
    info = venv / "lib" / "python3.12" / "site-packages" / f"isaacsim-{version}.dist-info"
    info.mkdir(parents=True)
    (info / "METADATA").write_text(f"Metadata-Version: 2.1\nName: isaacsim\nVersion: {version}\n")
    return python


def _sandbox(tmp_path):
    """A throwaway checkout (install.sh + env.py + an empty mcp/) with stub uv, claude,
    nvidia-smi and df, so nothing real is searched, installed or registered."""
    root = tmp_path / "ws" / "twinsmith"
    (root / "scripts").mkdir(parents=True)
    (root / "mcp").mkdir()
    for rel in ("install.sh", "scripts/env.py"):
        (root / rel).write_bytes((REPO / rel).read_bytes())
    home = tmp_path / "home"
    (home / ".local" / "bin").mkdir(parents=True)
    stubs = tmp_path / "stubs"
    stubs.mkdir()
    log = tmp_path / "calls.log"
    scripts = {
        "uv": f'echo "uv $*" >> "{log}"\n'
              f'echo "UV_PROJECT_ENVIRONMENT=$UV_PROJECT_ENVIRONMENT" >> "{log}"\n'
              'mkdir -p "$HOME/.local/bin"\n'
              'printf "#!/bin/sh\\nexit 0\\n" > "$HOME/.local/bin/twinsmith"\n'
              'chmod +x "$HOME/.local/bin/twinsmith"\n'
              'prev=\n'
              'for arg in "$@"; do\n'
              '  if [ "$prev" = "--project" ]; then\n'
              '    mkdir -p "$arg/.venv/bin"\n'
              '    printf "#!/bin/sh\\nexit 0\\n" > "$arg/.venv/bin/twinsmith"\n'
              '    chmod +x "$arg/.venv/bin/twinsmith"\n'
              '  fi\n'
              '  prev=$arg\n'
              'done\n',
        "claude": f'echo "claude $*" >> "{log}"\n',
        "nvidia-smi": 'echo "NVIDIA RTX PRO 6000 Blackwell, 595.99"\n',
        "df": 'printf "Avail\\n500G\\n"\n',
    }
    for name, body in scripts.items():
        (stubs / name).write_text("#!/bin/sh\n" + body)
        (stubs / name).chmod(0o755)
    env = {k: v for k, v in os.environ.items()
           if k not in {"TWINSMITH_SIM_PYTHON", "ISAACSIM_PATH",
                        "ISAAC_PATH", "CONDA_PREFIX", "VIRTUAL_ENV",
                        "TWINSMITH_WORKSPACE", "OMNI_KIT_ACCEPT_EULA"}}
    env |= {"HOME": str(home), "PATH": f"{stubs}{os.pathsep}{os.environ['PATH']}"}
    return root, env, log


def _install(root, env, *args):
    return subprocess.run(["bash", str(root / "install.sh"), *args], env=env, capture_output=True, text=True)


def _calls(log):
    return log.read_text() if log.exists() else ""


def _sim_synced(log):
    """The ~20 GB Isaac Sim environment sync, not the host project's locked sync."""
    return "uv sync --python 3.12" in _calls(log)


def _host_locked(log):
    text = _calls(log)
    return ("uv sync --project" in text and "--locked" in text
            and "--extra isaaclab" in text and "--no-dev" in text)


def test_an_explicitly_named_isaac_sim_is_used_and_saved_for_the_client(tmp_path):
    root, env, log = _sandbox(tmp_path)
    env["TWINSMITH_SIM_PYTHON"] = str(_fake_isaac(tmp_path / "lab-isaac", "6.0.0"))
    result = _install(root, env, "--claude-code")
    assert result.returncode == 0, result.stdout + result.stderr
    assert "using the Isaac Sim already installed" in result.stdout
    assert not _sim_synced(log) and _host_locked(log)  # no ~20 GB download
    assert "not the tested 6.2 release" in result.stderr
    assert f"TWINSMITH_SIM_PYTHON={env['TWINSMITH_SIM_PYTHON']}" in _calls(log)  # launch_simulator will find it


def test_an_isaac_sim_next_to_the_checkout_is_found_without_any_variable(tmp_path):
    root, env, log = _sandbox(tmp_path)
    python = _fake_isaac(root.parent / "other-project" / ".venv", "6.2.0.0")
    result = _install(root, env, "--claude-code")
    assert result.returncode == 0, result.stdout + result.stderr
    assert str(python) in result.stdout and not _sim_synced(log) and _host_locked(log)
    assert f"TWINSMITH_SIM_PYTHON={python}" in _calls(log)


def test_without_any_isaac_sim_the_locked_environment_is_downloaded(tmp_path):
    import glob

    if glob.glob("/opt/nvidia/isaac-sim*") or Path("/isaac-sim").exists():
        pytest.skip("this machine has a system-wide Isaac Sim, which the search always sees")
    root, env, log = _sandbox(tmp_path)
    result = _install(root, env, "--accept-eula")
    assert result.returncode == 0, result.stdout + result.stderr
    assert _sim_synced(log) and _host_locked(log)


def test_download_sim_downloads_even_with_an_existing_isaac_sim(tmp_path):
    root, env, log = _sandbox(tmp_path)
    env["TWINSMITH_SIM_PYTHON"] = str(_fake_isaac(tmp_path / "lab-isaac", "6.2.0.0"))
    result = _install(root, env, "--accept-eula", "--download-sim")
    assert result.returncode == 0, result.stdout + result.stderr
    assert "using the Isaac Sim already installed" not in result.stdout
    assert _sim_synced(log) and _host_locked(log)


@pytest.mark.parametrize("value", ["/no/such/python", "/usr/bin/python3"])
def test_a_named_install_that_is_not_isaac_sim_6_stops_instead_of_downloading(tmp_path, value):
    root, env, log = _sandbox(tmp_path)
    env["TWINSMITH_SIM_PYTHON"] = value
    result = _install(root, env, "--accept-eula")
    assert result.returncode == 1 and "does not point at an Isaac Sim 6.x" in result.stderr
    assert not _sim_synced(log) and not _host_locked(log)


def test_install_refuses_no_sim_with_download_sim(tmp_path):
    root, env, _ = _sandbox(tmp_path)
    result = _install(root, env, "--no-sim", "--download-sim")
    assert result.returncode == 2 and "cannot be combined" in result.stderr


def _fake_standalone(root: Path, version: str) -> Path:
    """A standalone install root (python.sh + VERSION), as NVIDIA's zip/workstation install."""
    root.mkdir(parents=True)
    (root / "python.sh").write_text("#!/bin/sh\nexit 0\n")
    (root / "python.sh").chmod(0o755)
    (root / "VERSION").write_text(f"{version}\n")
    return root


def test_a_valid_standalone_isaacsim_path_is_used(tmp_path):
    # The README's ISAACSIM_PATH advice must work.
    root, env, log = _sandbox(tmp_path)
    env["ISAACSIM_PATH"] = str(_fake_standalone(tmp_path / "isaac-sim", "6.2.0"))
    result = _install(root, env, "--claude-code")
    assert result.returncode == 0, result.stdout + result.stderr
    assert "standalone:ISAACSIM_PATH" in result.stdout and not _sim_synced(log) and _host_locked(log)
    assert "TWINSMITH_SIM_PYTHON=" in _calls(log)


def test_a_typo_is_reported_even_when_another_environment_is_active(tmp_path):
    # An activated venv must not silently replace what the user named.
    root, env, log = _sandbox(tmp_path)
    env["TWINSMITH_SIM_PYTHON"] = "/no/such/python"
    env["VIRTUAL_ENV"] = str(_fake_isaac(tmp_path / "active-venv", "6.2.0.0").parent.parent)
    result = _install(root, env, "--claude-code")
    assert result.returncode == 1 and "TWINSMITH_SIM_PYTHON is set but does not point" in result.stderr
    assert not _sim_synced(log) and not _host_locked(log) and "claude mcp add" not in _calls(log)


def test_the_opencode_snippet_is_valid_json_for_unusual_paths(tmp_path):
    import json
    import re

    root, env, _ = _sandbox(tmp_path)
    env["TWINSMITH_SIM_PYTHON"] = str(_fake_isaac(tmp_path / 'odd"name\\dir', "6.2.0.0"))
    result = _install(root, env, "--print-opencode")
    assert result.returncode == 0, result.stdout + result.stderr
    body = re.search(r'"environment": (\{.*\})', result.stdout).group(1)
    assert json.loads(body)["TWINSMITH_SIM_PYTHON"] == env["TWINSMITH_SIM_PYTHON"]


def test_no_sim_records_an_installed_isaac_sim_and_never_syncs(tmp_path):
    root, env, log = _sandbox(tmp_path)
    env["TWINSMITH_SIM_PYTHON"] = str(_fake_isaac(tmp_path / "lab-isaac", "6.2.0.0"))
    env["OMNI_KIT_ACCEPT_EULA"] = "YES"
    result = _install(root, env, "--no-sim", "--claude-code")
    assert result.returncode == 0, result.stdout + result.stderr
    assert not _sim_synced(log) and _host_locked(log)
    assert f"TWINSMITH_SIM_PYTHON={env['TWINSMITH_SIM_PYTHON']}" in _calls(log)


def test_no_sim_without_any_isaac_sim_never_syncs_the_simulator_even_with_the_eula_accepted(tmp_path):
    root, env, log = _sandbox(tmp_path)
    env["OMNI_KIT_ACCEPT_EULA"] = "YES"
    result = _install(root, env, "--no-sim", "--claude-code")
    assert result.returncode == 0, result.stdout + result.stderr
    assert not _sim_synced(log) and _host_locked(log) and "TWINSMITH_SIM_PYTHON=" not in _calls(log)
    assert "no Isaac Sim 6.x found to record" in result.stdout


def test_no_sim_with_a_bad_named_interpreter_warns_and_still_installs_the_host(tmp_path):
    root, env, log = _sandbox(tmp_path)
    env["TWINSMITH_SIM_PYTHON"] = "/no/such/python"
    result = _install(root, env, "--no-sim", "--claude-code")
    assert result.returncode == 0, result.stdout + result.stderr
    assert "TWINSMITH_SIM_PYTHON is set but does not point" in result.stderr
    assert "no interpreter recorded" in result.stderr
    assert not _sim_synced(log) and _host_locked(log) and "TWINSMITH_SIM_PYTHON=" not in _calls(log)


def test_opencode_entry_uses_the_installed_path_so_it_starts_without_path_setup(tmp_path):
    root, env, log = _sandbox(tmp_path)
    result = _install(root, env, "--no-sim", "--print-opencode")
    assert result.returncode == 0, result.stdout + result.stderr
    assert f'"command": ["{Path(env["HOME"]) / ".local" / "bin" / "twinsmith"}"]' in result.stdout


def test_an_explicitly_named_51_is_used_with_a_clear_warning(tmp_path):
    root, env, log = _sandbox(tmp_path)
    env["TWINSMITH_SIM_PYTHON"] = str(_fake_isaac(tmp_path / "bricksim", "5.1.0.0"))
    result = _install(root, env, "--claude-code")
    assert result.returncode == 0, result.stdout + result.stderr
    assert not _sim_synced(log) and _host_locked(log)
    assert "explicitly pinned older build" in result.stderr and "auto-detection never picks it" in result.stderr
    assert f"TWINSMITH_SIM_PYTHON={env['TWINSMITH_SIM_PYTHON']}" in _calls(log)


def test_auto_detection_never_picks_51(tmp_path):
    root, env, log = _sandbox(tmp_path)
    old = _fake_isaac(root.parent / "older-project" / ".venv", "5.1.0.0")
    new = _fake_isaac(root.parent / "other-project" / ".venv", "6.2.0.0")
    result = _install(root, env, "--claude-code", "--no-sim")
    assert result.returncode == 0, result.stdout + result.stderr
    assert f"TWINSMITH_SIM_PYTHON={new}" in _calls(log) and str(old) not in _calls(log)


def test_a_named_older_build_other_than_51_is_still_refused(tmp_path):
    root, env, _ = _sandbox(tmp_path)
    env["TWINSMITH_SIM_PYTHON"] = str(_fake_isaac(tmp_path / "old", "4.5.0"))
    result = _install(root, env)
    assert result.returncode == 1 and "6.x or 5.1 install" in result.stderr


@pytest.mark.parametrize("port", ["abc", "0", "70000", "80 80", ""])
def test_a_bad_port_stops_before_anything_is_installed(tmp_path, port):
    root, env, log = _sandbox(tmp_path)[:3]
    result = _install(root, env, "--no-sim", "--port", port)
    assert result.returncode == 2 and "port must be a number" in result.stderr


def test_host_sync_is_locked_into_the_checkout_and_not_the_simulator_venv(tmp_path):
    root, env, log = _sandbox(tmp_path)
    result = _install(root, env, "--no-sim")
    assert result.returncode == 0, result.stdout + result.stderr
    assert not _sim_synced(log) and _host_locked(log)
    venv = root / "mcp" / ".venv"
    assert f"UV_PROJECT_ENVIRONMENT={venv}" in _calls(log)
    assert f"uv sync --project {root / 'mcp'} --locked --extra isaaclab --no-dev" in _calls(log)
    assert (venv / "bin" / "twinsmith").is_file()
    link = Path(env["HOME"]) / ".local" / "bin" / "twinsmith"
    assert link.is_symlink() and link.resolve() == (venv / "bin" / "twinsmith").resolve()


def test_a_claude_desktop_config_it_cannot_parse_is_left_alone_with_the_entry_to_add(tmp_path):
    root, env = _sandbox(tmp_path)[:2]
    config = Path(env["HOME"]) / ".config/Claude/claude_desktop_config.json"
    config.parent.mkdir(parents=True)
    config.write_text('{ // my comment\n"mcpServers": {} }\n')
    result = _install(root, env, "--no-sim", "--claude-desktop")
    assert result.returncode == 0 and "not changed" in result.stderr and '"twinsmith"' in result.stderr
    assert "Traceback" not in result.stderr and "my comment" in config.read_text()


def _kit_config(home, app, version):
    path = home / ".local/share/ov/data/Kit" / app / version / "user.config.json"
    path.parent.mkdir(parents=True)
    path.write_text("{}")
    return path


def _run_install_py(home, *args):
    return subprocess.run([sys.executable, str(REPO / "scripts" / "install.py"), *args],
                          env=os.environ | {"HOME": str(home)}, capture_output=True, text=True)


def test_kit_registration_refuses_to_guess_between_configs(tmp_path):
    old = _kit_config(tmp_path, "Isaac-Sim Full", "5.1")
    new = _kit_config(tmp_path, "Isaac-Sim Full", "6.2")
    result = _run_install_py(tmp_path, "status")
    assert result.returncode != 0
    assert "pass the one you mean with --config" in result.stderr
    assert str(old) in result.stderr and str(new) in result.stderr
    chosen = _run_install_py(tmp_path, "status", "--config", str(new))
    assert chosen.returncode == 0 and str(new) in chosen.stdout


def test_kit_registration_uses_a_lone_config(tmp_path):
    only = _kit_config(tmp_path, "Isaac-Sim Full", "6.2")
    result = _run_install_py(tmp_path, "status")
    assert result.returncode == 0 and str(only) in result.stdout


@pytest.mark.parametrize("flag", ["--help"])
def test_run_help_does_not_present_a_missing_interpreter_as_chosen(tmp_path, flag):
    missing = tmp_path / "no-such-python"
    env = os.environ | {"TWINSMITH_SIM_PYTHON": str(missing)}
    result = subprocess.run(["bash", str(REPO / "run.sh"), flag], env=env, capture_output=True, text=True)
    assert result.returncode == 0
    assert f"Interpreter: {missing} (not found;" in result.stdout


def test_install_help_describes_the_project_not_one_robot():
    result = subprocess.run(["bash", str(REPO / "install.sh"), "--help"], capture_output=True, text=True)
    assert result.returncode == 0
    assert "UR5e digital twin repository" not in result.stdout
    assert "Never download or sync Isaac Sim" in result.stdout
