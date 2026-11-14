# Plans

One specification per upcoming pull request. Each is written to be handed to a fresh agent thread as its whole brief: the
problem, the design, the files, the tests and the checks that say it is done. The phases they serve are in the
[roadmap](../ROADMAP.md); the layout they build is in [How it works](../ARCHITECTURE.md#target-layout-planned).

| Order | Plan | Adds | Needs the ASU PC's GPU or the private pack? |
|--:|---|---|---|
| 3 | [03-planning-eval-wrappers](03-planning-eval-wrappers.md) | `gap` and `vabar` | No for the pull request; yes to use it |
| 4 | [04-robosim-bump-and-sysid](04-robosim-bump-and-sysid.md) | RoboSimStudio at a newer pin, `sysid` | No |

Plans 3 and 4 are independent of each other. Plan 1 does not wait for plan 2.

**Hosts.** Every live, GPU or wrapped-tool step in these plans runs on the ASU PC, which stays on. No step may require the
home PC. The ASU PC's GPU is shared with Jenga games: check for a running game first, never disturb one, and use the GPU
guards. Its workspace is still the old folder, so name it with `TWINSMITH_WORKSPACE`. Details and limits:
[hardware and hosts](../ROADMAP.md#hardware-and-hosts).

## Rules for whoever executes a plan

- Read [AGENTS.md](../../AGENTS.md) first. Do not install or start Isaac Sim, and do not start GPU work, unless the plan's part
  says the owner runs it.
- One plan, one pull request, opened as a draft against `main`. Do not merge it.
- Lean: delete what the change makes unused, in the same pull request. No alias for a renamed thing, no compatibility shim.
- After changing a tool's registration or contract, run `python3 scripts/gen_host_catalog.py --write`, then
  `python3 scripts/gen_tool_reference.py --write`, and commit the results.
- Before pushing, run what CI runs: `python3 scripts/check_docs.py`, `python3 scripts/check_names.py --enforce`,
  `python3 scripts/check_assets.py`, `python3 scripts/check_portable_paths.py`, `python3 scripts/gen_host_catalog.py --check`,
  and from `mcp/`: `uv run --locked --isolated --python 3.12 --extra test --extra isaaclab --extra scene-test pytest -q`.
- A new environment variable goes in [ENVIRONMENT.md](../ENVIRONMENT.md); a test keeps that page equal to the code.
- When the plan's tools are registered, move its tool names from the "Proposed tools" table in
  [How it works](../ARCHITECTURE.md#target-layout-planned) into the page's description of what exists, update the
  [roadmap](../ROADMAP.md) state table, and delete the plan file. A finished plan is not kept.
- Say what was verified and what was not. A passing host suite is not a live simulator result.

Pages in this folder are outside `scripts/check_docs.py`'s page list, which is why they may name tools that are not
registered yet. Reference pages may not.
