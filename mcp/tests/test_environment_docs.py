"""docs/ENVIRONMENT.md lists every environment variable the code reads, once, and nothing else.

"Read by the code" means: an `os.environ`/`os.getenv`-style read of any name in Python, a project variable
(`TWINSMITH_*`, `ISAAC_*`, `ISAACLAB_*`) named as a string constant in Python, or a shell expansion in `run.sh`,
`install.sh` or `scripts/*.sh`. Ordinary system variables (`PATH`, `HOME`, ...) are not listed; the few the tools document
are in `SYSTEM_VARIABLES`.
"""

import ast
import re
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
PAGE = ROOT / "docs" / "ENVIRONMENT.md"

PROJECT_PREFIXES = ("TWINSMITH_", "ISAAC_", "ISAACLAB_", "ISAACSIM_")
# System variables the tools read on purpose and the page documents. Every other system variable is read as usual.
SYSTEM_VARIABLES = frozenset({"DISPLAY", "ROS_DOMAIN_ID", "ROS_DISTRO", "OMNI_KIT_ACCEPT_EULA", "XDG_RUNTIME_DIR",
                              "XDG_STATE_HOME", "CUDA_VISIBLE_DEVICES"})
# Project-looking names that are not settings, with the reason.
NOT_SETTINGS = {
    "TWINSMITH_SIM_SESSION_ID": "exported to a managed session's Kit, read by no code",
    "TWINSMITH_SIM_SESSION_DIR": "exported to a managed session's Kit, read by no code",
    "TWINSMITH_REPO": "exported to the scan step's multi-view cycle (the puzzle project's), read by no code here",
    "TWINSMITH_SCAN_REPO": "exported to the scan step's multi-view cycle (the puzzle project's), read by no code here",
}
SOURCE_DIRS = ("mcp/twinsmith", "mcp/templates", "exts", "scripts")
SKIP_FILES = {"scripts/check_names.py", "scripts/check_private_paths.py", "scripts/check_portable_paths.py"}
NAME = re.compile(r"[A-Z][A-Z0-9_]+")
SHELL_VARIABLE = re.compile(r"\$\{?([A-Z][A-Z0-9_]+)")


def is_project(name):
    return name.startswith(PROJECT_PREFIXES) and not name.endswith("_") and name != "ISAAC_SIM"


def environment_like(node):
    text = ast.unparse(node)
    return "environ" in text or text.rsplit(".", 1)[-1] in {"env", "environment", "child_env", "base_env"}


def python_reads(tree):
    """(names read from an environment mapping, project names appearing as string constants)."""
    reads, constants = set(), set()
    documented = {id(node.value) for node in ast.walk(tree)
                  if isinstance(node, ast.Expr) and isinstance(node.value, ast.Constant)}
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) and node.func.attr in {"get", "pop", "setdefault"}:
            if node.args and isinstance(node.args[0], ast.Constant) and environment_like(node.func.value):
                reads.add(node.args[0].value)
        elif isinstance(node, ast.Call) and ast.unparse(node.func).endswith("getenv") and node.args \
                and isinstance(node.args[0], ast.Constant):
            reads.add(node.args[0].value)
        elif isinstance(node, ast.Subscript) and environment_like(node.value) and isinstance(node.slice, ast.Constant):
            reads.add(node.slice.value)
        elif isinstance(node, ast.Compare) and isinstance(node.left, ast.Constant) \
                and any(isinstance(op, (ast.In, ast.NotIn)) for op in node.ops) \
                and any(environment_like(c) for c in node.comparators):
            reads.add(node.left.value)
        elif isinstance(node, ast.Constant) and isinstance(node.value, str) and id(node) not in documented \
                and NAME.fullmatch(node.value) and is_project(node.value):
            constants.add(node.value)
    return {name for name in reads if isinstance(name, str) and NAME.fullmatch(name)}, constants


def read_variables(root=ROOT):
    """Every variable the code under `root` reads, split into project variables and system variables."""
    names = set()
    sources = [path for directory in SOURCE_DIRS if (root / directory).is_dir()
               for path in sorted((root / directory).rglob("*")) if path.is_file()]
    sources += [root / name for name in ("run.sh", "install.sh") if (root / name).is_file()]
    for path in sources:
        relative = path.relative_to(root).as_posix()
        if "tests" in path.parts or "__pycache__" in path.parts or relative in SKIP_FILES:
            continue
        if path.suffix == ".py":
            reads, constants = python_reads(ast.parse(path.read_text(encoding="utf-8")))
            names |= reads | constants
        elif path.suffix == ".sh" or path.name in {"run.sh", "install.sh"}:
            names |= set(SHELL_VARIABLE.findall(path.read_text(encoding="utf-8")))
    return names


def required(names):
    """The variables the page must list."""
    return {name for name in names if (is_project(name) or name in SYSTEM_VARIABLES)} - set(NOT_SETTINGS)


def documented(text):
    """{variable: how many table rows list it}, from the first cell of every table row."""
    counts = {}
    for line in text.splitlines():
        if not line.startswith("| `"):
            continue
        first = line.split("|")[1]
        for name in re.findall(r"`([A-Z][A-Z0-9_]+)`", first):
            counts[name] = counts.get(name, 0) + 1
    return counts


def test_every_variable_the_code_reads_is_listed():
    missing = sorted(required(read_variables()) - set(documented(PAGE.read_text(encoding="utf-8"))))
    assert not missing, f"docs/ENVIRONMENT.md does not list: {', '.join(missing)}"


def test_every_listed_variable_is_read_by_the_code():
    read = read_variables()
    unread = sorted(name for name in documented(PAGE.read_text(encoding="utf-8")) if name not in read)
    assert not unread, f"docs/ENVIRONMENT.md lists variables that no code reads: {', '.join(unread)}"


def test_each_variable_is_listed_once():
    twice = sorted(name for name, count in documented(PAGE.read_text(encoding="utf-8")).items() if count > 1)
    assert not twice, f"listed more than once: {', '.join(twice)}"


def test_every_row_names_a_default_a_reader_and_a_meaning():
    rows = [line for line in PAGE.read_text(encoding="utf-8").splitlines()
            if line.startswith("| `") and re.search(r"`[A-Z][A-Z0-9_]+`", line.split("|")[1])]
    assert rows
    for line in rows:
        cells = [cell.strip() for cell in line.strip().strip("|").split("|")]
        assert len(cells) == 4 and all(cells), line


def test_the_scanner_finds_reads_and_skips_the_rest(tmp_path):
    (tmp_path / "mcp/twinsmith").mkdir(parents=True)
    (tmp_path / "mcp/twinsmith/a.py").write_text(
        '"""TWINSMITH_IN_A_DOCSTRING mention."""\n'
        "import os\n"
        'x = os.environ.get("TWINSMITH_ONE") or os.getenv("SOME_OTHER")\n'
        'y = os.environ["ROS_DOMAIN_ID"]\n'
        'KNOWN = "ISAACLAB_TWO"\n'
        'PREFIX = "TWINSMITH_"\n'
        'if "TWINSMITH_THREE" in os.environ:\n    pass\n'
        'z = "PATH"\n')
    (tmp_path / "run.sh").write_text('PORT="${TWINSMITH_PORT:-8766}"\necho "$HOME $TWINSMITH_FOUR"\n')
    (tmp_path / "mcp/twinsmith/tests").mkdir()
    (tmp_path / "mcp/twinsmith/tests/skipped.py").write_text('import os\nos.environ.get("TWINSMITH_TEST_ONLY")\n')
    names = read_variables(tmp_path)
    assert {"TWINSMITH_ONE", "SOME_OTHER", "ROS_DOMAIN_ID", "ISAACLAB_TWO", "TWINSMITH_THREE",
            "TWINSMITH_PORT", "TWINSMITH_FOUR", "HOME"} <= names
    assert not {"TWINSMITH_IN_A_DOCSTRING", "TWINSMITH_", "PATH", "TWINSMITH_TEST_ONLY"} & names
    assert required(names) == {"TWINSMITH_ONE", "ROS_DOMAIN_ID", "ISAACLAB_TWO", "TWINSMITH_THREE", "TWINSMITH_PORT",
                               "TWINSMITH_FOUR"}


def test_a_missing_or_unread_variable_fails():
    text = PAGE.read_text(encoding="utf-8")
    without = "\n".join(line for line in text.splitlines() if not line.startswith("| `TWINSMITH_TOKEN`"))
    assert "TWINSMITH_TOKEN" in required(read_variables()) - set(documented(without))
    invented = text + "\n| `TWINSMITH_NO_SUCH_SETTING` | unset | host | invented |\n"
    assert "TWINSMITH_NO_SUCH_SETTING" not in read_variables()
    assert "TWINSMITH_NO_SUCH_SETTING" in documented(invented)
