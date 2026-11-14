# Planning and evaluation: `gap` and `vabar`

Two wrapped tools, each in its own toolset and both off by default:

| Toolset | Tool | Wraps | Licence |
|---|---|---|---|
| `planning` | `gap` | [Graph-as-Policy](https://github.com/graph-robots/graph-as-policy): compiles a task into a typed skill graph and runs it in simulation | Apache-2.0 |
| `eval` | `vabar` | [VABAR](https://github.com/BerkeleyAutomation/VABAR): benchmark task variants and frozen cells on Newton through RoboSimStudio | Apache-2.0 |

Neither needs an Isaac Sim or Kit bridge. Each runs the upstream command line from a pinned checkout with that checkout's
own interpreter. twinsmith never imports, installs or updates either one, and copies nothing from them. Turn them on with
`toolsets(action="enable", names=["planning", "eval"])` or `TWINSMITH_TOOLSETS=default,+planning,+eval`.

## Setup (the owner runs this)

One Graph-as-Policy checkout serves both tools, at the commit VABAR 1.0 names. The three checkouts sit side by side under
`<workspace>/tools/`:

```text
<workspace>/tools/
  graph-as-policy/      KeplerC/graph-as-policy at 06dd6193e13eab816123febef1369deda0849d91
  open-robot-skills/    KeplerC/open-robot-skills at b15859a244658abc6060f3aa3b6827780746a5fe
  vabar/                BerkeleyAutomation/VABAR at 64b594ed2e1ae25e431d235de086c7cfd99d9aaf ("VABAR 1.0 release")
```

```bash
cd <workspace>/tools
git clone https://github.com/KeplerC/graph-as-policy.git
git -C graph-as-policy checkout 06dd6193e13eab816123febef1369deda0849d91
git -C graph-as-policy submodule update --init --recursive
git clone https://github.com/KeplerC/open-robot-skills.git
git -C open-robot-skills checkout b15859a244658abc6060f3aa3b6827780746a5fe
git clone https://github.com/BerkeleyAutomation/VABAR.git vabar
git -C vabar checkout 64b594ed2e1ae25e431d235de086c7cfd99d9aaf
```

Then create each environment with the steps in upstream's READMEs at those commits. Both need an NVIDIA GPU and CUDA:
Graph-as-Policy's base dependencies include cuRobo and `cuda-core[cu12]`, and VABAR's include `robosimstudio[gap,curobo]`
(about 12 GB). There is no CPU-only environment.

```bash
(cd graph-as-policy && uv sync && uv run gap skills install --all && uv run gap skills check --download)
(cd open-robot-skills/tools/curobo && CUDA_HOME=/usr/local/cuda uv sync)   # cuRobo tool bundle, once
(cd vabar && uv sync && uv run python -m vabar list && uv run python -m vabar warmup)
```

`gap skills check --download` needs `HF_TOKEN`, and the SAM 3 weights are gated: the Hugging Face account behind the
token must also be granted access at `https://huggingface.co/facebook/sam3`. Eight of VABAR's twelve bench cells fail
without it: their reference graphs, `gap_perception` (tool hanging, gear meshing, syringe packing) and `gap_perception_loop`
(tool sorting), call `sam3.segment_text`. The four cable cells run with upstream's palette segmenter, which `bench` selects
(`ROBOSIMSTUDIO_SEGMENTATION_BACKEND=palette`); a plain `run` of a cable graph does not set it, and the runtime then picks
SAM 3. The routines do not use it. VABAR's `warmup` compiles the renderer's
shader cache and Warp kernels once, in 5 to 8 minutes; a graph cell's first run fails without it. Graph-as-Policy asks
for 24 GB of GPU memory: run it on a PC that has that. `gap(action="status")` reports a smaller GPU rather than working
around it.

Other locations: `TWINSMITH_GAP_ROOT` and `TWINSMITH_VABAR_ROOT` (see [ENVIRONMENT.md](ENVIRONMENT.md)). The skill
library is always the `open-robot-skills` folder beside the Graph-as-Policy checkout.

## Paid model calls

`gap(action="generate")` asks a paid model (OpenRouter by default) to write a graph. It is refused unless the owner's
own file allows it:

```toml
# ~/.config/twinsmith/config.toml
[planning]
allow_paid = true
```

A project `twinsmith.toml` cannot set it. Without it, the tools also keep `OPENROUTER_API_KEY`, `GAP_VLM_*` and `GAP_LLM_*`
from every process they start, so graph nodes that ask a model fail instead of spending money (the answer says so), and
`check` reports the key as missing. `HF_TOKEN` is always passed, because model weights are free downloads, and so are
these settings when they are set: `HF_HOME`, `GAP_VAB_ROOT`, `GAP_LIBERO_PRO_ROOT`, `GAP_SAM3_DEVICE`, `GAP_DINO_DEVICE`,
`GAP_DINO_MODEL` and `GAP_MOLMO_MODEL`. `GAP_MOLMO_BASE_URL` passes only when it points at this machine; otherwise it is
dropped with a warning. `status` reports whether each key is set and never its value. A malformed `config.toml` or a
`[planning]` value of the wrong type is refused with `CONFIG_INVALID`; nothing runs.

### A VLM endpoint for graph nodes

Graph nodes that ask a VLM can use an endpoint the owner names, without `allow_paid`: a hosted OpenAI-compatible API
(NVIDIA's `https://integrate.api.nvidia.com/v1`, a university gateway) or a server on this machine.

```toml
# ~/.config/twinsmith/config.toml
[planning]
vlm_url = "https://integrate.api.nvidia.com/v1"   # https to any host, or http only to localhost, 127.0.0.0/8 or ::1
vlm_model = "<model id>"                          # optional; the endpoint's model name
vlm_api_key_file = "~/.config/agent-secrets/nvidia_api_key"   # optional
```

The tools then set `GAP_VLM_PROVIDER=openrouter` (upstream's OpenAI-compatible client), `GAP_VLM_BASE_URL`,
`GAP_VLM_MODEL` and, from the key file, `GAP_VLM_API_KEY` on the `gap` and `vabar` processes. They never read these
from the environment, and these settings override any `GAP_VLM_*` that `allow_paid` let through. The endpoint gets only
its own key: upstream's client falls back to `OPENROUTER_API_KEY`, so with an endpoint set neither that nor
`GAP_VLM_API_KEY` reaches a run, even with `allow_paid`. `generate`'s planner is a different client: it gets no
endpoint, and its `OPENROUTER_API_KEY` still needs `allow_paid`.

- The URL must have a host, and no user, password, query or fragment in it (a key goes in the key file). Anything else
  is `CONFIG_INVALID` for `run` and `check`; `status` reports the problem instead of refusing, and the other actions
  never read these settings.
- The key file must be a regular file (not a link), owned by the owner and readable by no one else (`chmod 600`),
  holding one key on one line. It is read when a process starts. Answers, job records and `status` never contain the key.
- `status` reports the URL, host and model, the key file's path and whether it is usable.
- `check` sees the endpoint and model, not the key, so it reports what a run will use. `validate`, `tools` and
  `vabar list` ask no model and get neither.

The owner chooses the endpoint and pays whatever it costs: a free tier's limits, or a paid API behind a local server,
are theirs. Graph nodes send the endpoint their prompts and camera images. Only VLM nodes use it; `generate`'s planner is
a separate model and still needs `allow_paid`.

## `gap`

| Action | Runs | Notes |
|---|---|---|
| `status` | Nothing upstream | Checkout, commit and pin; the skill library's commit and pin; GPU memory against the 24 GB requirement; whether `HF_TOKEN` and `OPENROUTER_API_KEY` are set; `allow_paid` |
| `tools` | `gap tools list --format json` | The skill catalog. Read-only |
| `check` | `gap check --format json` | Which tool bundles are ready here and which skills can run. Read-only; no `--probe`, so no model is called |
| `validate` | `gap run --validate-only --skills <library> <graph>` | Structure, types and skill contracts of a graph, with no simulator, GPU, model or job; returns the issues and a verdict. Upstream imports each script node's module to read its signature, so a graph with Python that is not trusted (see the safety rule) and not allowed is not handed to it: the answer lists its nodes, scripts and `*.py` files instead, to read before setting `allow_scripts` |
| `generate` | `gap generate <instruction> --out ...` | A job, no GPU. Needs `allow_paid`. The graph lands in `<workspace>/runs/gap/<name>/output/` |
| `run` | `gap run --sim <SUITE/TASK> ... [--checkpoints off] <graph>` | A job that uses the GPU. `graph` is a workflow directory or `workflow.json` in `<workspace>/runs/gap/` (a relative path is read from there), the Graph-as-Policy checkout or the VABAR checkout. Traces go to `<workspace>/runs/gap/<name>/output/` |

At the pin, `gap check` takes no graph: it is the capability report; `validate` checks one graph.

### Author a graph with your coding agent

`generate` is upstream's own planner and needs a paid model. A coding agent you already use (Claude Code, Codex) can
write the graph instead. Upstream's own agent skill says so: inside Claude Code the agent is the code generator and
should not call `gap generate`, but author the graph and drive `gap run --validate-only` to a clean pass
(`agent/skills/gap/SKILL.md`, section 3, at the pin). Then:

1. Write `<workspace>/runs/gap/<name>/workflow.json`, plus any `scripts/` and `checkpoints/` files, from the schema in
   `docs/source/reference/workflow-schema.md` (hand-written JSON is a supported way to author a graph), the digest in
   `agent/skills/gap/references/authoring-graphs.md` and the example in `examples/libero_quickstart/graph/`. All three are in
   the Graph-as-Policy checkout. `gap(action="tools")` lists the skills a node can call.
2. `gap(action="validate", graph="<name>")` until it is valid. A graph with no Python is validated straight away; one with
   scripts returns the files to read, and the owner turns on `allow_scripts` once they have.
3. `gap(action="run", sim="<SUITE/TASK>", graph="<name>")` to simulate it.

Subscription CLIs (`claude -p`, `codex exec`) are not a supported model provider for `generate` or for graph nodes: the
providers' terms point automation at API keys.

**Safety rule: twinsmith does not run Graph-as-Policy on real hardware.** Upstream's `gap run` has `--real franka` and
`--real ur_zed`. The tool has no argument that produces `--real`, no free-form argument and no `extra_args`; `sim` is
required for `run` and must be `SUITE/TASK`. Every value is checked for `--real` (or an abbreviation argparse would expand
to it) before anything else, and the built argument list is scanned again before the process starts. Either refusal is
`REAL_ROBOT_REFUSED`, "twinsmith does not run Graph-as-Policy on real hardware". The process never sees `ROS_DOMAIN_ID`
or any other `ROS_*` variable.

That guard covers the command line only. Graph-as-Policy runs Python from a graph's folder in two ways: `script` and
`router` nodes, and `checkpoints/<subgraph>.py`, which it executes after each subgraph. Either can do anything the user
can. So before `run` starts, twinsmith reads the `workflow.json` (with its `subgraphs`) and the graph folder:

- every `script` path must be relative and resolve inside the graph's own folder (without `..`), or, for a graph that
  ships in the Graph-as-Policy or VABAR checkout, inside that checkout; otherwise the run is refused with
  `PATH_NOT_ALLOWED`;
- a graph in one of those checkouts runs as it is when the checkout is at its tested pin and git sees the graph folder
  and every script tracked and unmodified (nothing edited, untracked or ignored);
- any other graph with script or router nodes or `*.py` files (one in `<workspace>/runs/gap/`, such as a `generate`
  output with its checkpoint files, or a changed graph in a checkout) is refused with `SCRIPT_NOT_ALLOWED` unless the
  owner sets, in their own file:

```toml
# ~/.config/twinsmith/config.toml
[planning]
allow_scripts = true
```

Without it, a graph that passes (it has no Python) runs with `--checkpoints off`, so a checkpoint file added after the
check does not run. The graph is read again when the job starts, so a change to it in between is not seen by the
check; the folders are the owner's.

Like `allow_paid`, a project `twinsmith.toml` cannot set it. Read a generated graph's scripts before turning it on.

## `vabar`

| Action | Runs | Notes |
|---|---|---|
| `status` | Nothing upstream | Checkout, commit and pin; the commits and pins of the Graph-as-Policy and skill checkouts; whether `warmup` has run (its stamps in the renderer cache) |
| `list` | `python -m vabar list` | Every `suite/variant`. Read-only |
| `run` | `python -m vabar run --task <t> --solve routine\|graph\|idle [--graph <label>] [--episodes ...] [--bus-timeout <s>]` | A job that uses the GPU. `graph` is a label the task ships, such as `gt`. `bus_timeout_s` (30 to 3600) is how long a graph's tool call may wait. Upstream's default is 30 s; with `solve="graph"` and no value the tool passes what bench cells use, 180 s, or 600 s for the cable suites (`cable_route`, `usbc_insert`). A cold first run compiles Newton's IK kernel inside a tool call (44 s on the ASU PC), and `warmup` does not compile it |
| `bench` | `python -m vabar bench --cell <c> --solve ... [--limit n] --workers <1-4>` | `print_only=true` (the default) runs upstream's `--print` with the GPU hidden: it writes nothing and returns the resolved jobs (tasks, episodes, flags) or upstream's refusal of an unknown cell. `false` runs it as a job that uses the GPU; pass the same `name` to get the folder the plan printed (a name whose folder exists is refused either way) |
| `results` | Nothing upstream | Reads the `episode_results.jsonl` files in a run folder (at most 200, 8 MB each; links and files outside the folder are skipped) and returns successes per task and solver, with the three commonest reasons the others failed (upstream's `termination_reason`); without `run`, the newest run folders |

`warmup` and `studio` are not wrapped: the owner runs the one-time warmup, and `studio` serves a viewer on the network.
Run folders are `<workspace>/runs/vabar/<name>/`. Upstream has no real-hardware option at the pin (`run --help` and
`bench --help` read on 2026-11-13).

A `bench` with more than one worker starts a private NVIDIA MPS daemon (`vabar/mps.py`): only the run's own processes
connect to it, so other GPU jobs are not affected, and upstream stops it when the run ends. A job cancelled with
`jobs(action="cancel")` is killed before that cleanup can run, and the daemon (`nvidia-cuda-mps-control`) can be left
behind. Look for it after cancelling a multi-worker bench; it stops with `echo quit | CUDA_MPS_PIPE_DIRECTORY=<its pipe
folder> nvidia-cuda-mps-control`.

## Jobs, guards and answers

`generate`, `run` and `bench` return a `job_id` in the shared jobs folder: wait with `jobs(action="status", job_id=...,
wait_s=...)` and stop with `jobs(action="cancel")`. A job that uses the GPU is guarded as an Isaac Lab launch is: it is
refused beside another GPU job or a running Isaac Sim unless `allow_concurrent_gpu=true`, and past a configured GPU-hour
budget. Nothing is stopped to make room.

Every answer has the same envelope: `{ok, tool, action, commit, pin, pin_match, exit_code, result | error, warnings}`,
with `code` on a refusal (`REAL_ROBOT_REFUSED`, `PAID_NOT_ALLOWED`, `NOT_INSTALLED`, `PATH_NOT_ALLOWED`, `NOT_FOUND`,
`ALREADY_EXISTS`, `INVALID_VALUE`, `CONFIG_INVALID`, `SCRIPT_NOT_ALLOWED`, `GPU_BUSY`, `GPU_BUDGET`, `UPSTREAM_FAILED`). A checkout at another commit still runs, with a
warning that it differs from the tested pin.

## What has been verified

The host tests run both tools against fake checkouts: the exact argument lists, the refusals, the scrubbed environment,
the paid-call switch and the jobs.

On the ASU PC on 2026-11-13, with the three checkouts at their pins and their environments built (Graph-as-Policy on
Python 3.11, VABAR on 3.12; `gap skills check`: 43 bundles pass, the SAM 3 weights refused as above):

- **CPU, through the tools:** `gap` `status` (pins match, 97,887 MiB against 24 GB), `tools`, `check` and `validate`
  (`examples/libero_quickstart/graph`: trusted, 8 script nodes, `OK: 0 errors, 0 warning(s)`); `vabar` `status`
  (warmup ran), `list` (374 variants), `bench` with `print_only` (`tool_hanging/L1`: 4 jobs, 30 episodes; an unknown cell
  is refused with upstream's list of cells) and `results` on a real run folder.
- **CLI spellings:** `gap run`, `check`, `tools list` and `generate`, and `vabar run` and `bench`, read with `--help` at
  the pins; the argument lists the tools build match them. The `task`, `solution`, `success` and `termination_reason`
  fields `results` reads are the ones VABAR's own RoboSimStudio (`39b7a6e`) writes.
- **GPU, run once from an agent session, not by a test:** upstream's `warmup` on all six suites (18 to 247 s each), then,
  through the tool, `vabar run` of `tool_hanging/scissors_level1` with the `gt` graph: the first episode failed with
  `BusTimeout: build was not serviced within 30 s` (which is why `bus_timeout_s` exists), and a rerun succeeded, 1 of 1.

Not yet run: a `gap run`, a `generate`, a bench cell, and any graph that uses SAM 3.
