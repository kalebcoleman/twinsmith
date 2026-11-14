# Writing your own tools

A tool is a Python function the bridge runs on Kit's main loop. Put yours in an ordinary Python
package (not a Kit extension), and the bridge serves them beside its own: to MCP clients and to
scripts using the client.

## A tool package

```python
# my_tools/__init__.py
MCP_NAMESPACE = "robot_demo"            # default: the module's name

def register(registrar):
    @registrar.tool("Spawn a robot into the scene.", {"name": {"type": "string", "description": "Prim name."}})
    def spawn_robot(name: str = "robot_1"):
        import omni.usd                  # Kit imports go inside the function
        ...
        return {"path": f"/World/{name}"}
```

This registers `robot_demo.spawn_robot`, which an MCP client sees as `robot_demo__spawn_robot`.
Copy `mcp/templates/demo_tools/` for a working example.

A package with no `register` function is read instead: every public function defined in it (or
those named in `__all__`) becomes a tool, described by its docstring, with arguments taken from
its signature and type hints.

Load a package with `TWINSMITH_TOOL_PATHS=<folder holding it>` and `TWINSMITH_TOOL_MODULES=my_tools`
at launch, or store that once with
`python3 scripts/install.py add-project --path <folder> --module my_tools`. A package that fails
to import stops the bridge from starting and says why, so a missing tool is never a silent one.

## What a tool is given and returns

- Arguments arrive by name. Declare each one in the parameters: `type` (`string`, `integer`,
  `number`, `boolean`, `array`, `object`), `description`, and optionally `required`, `default`, `enum`.
- Return anything JSON can hold. Do not build the reply message yourself.
- To fail, raise. `twinsmith_bridge.ToolError(message, details={...})` sends the details to the
  caller; any other exception is sent with its traceback.
- A tool may be `async` and await Kit; the bridge awaits it.

Three flags on `registrar.tool(...)` change how a tool is scheduled
([when a call runs](protocol.md#when-a-call-runs)):

| Flag | Meaning |
|---|---|
| `read_only=True` | Changes nothing. The only tools that run in read-only mode. |
| `exclusive=True` | Advances the app itself (awaits frames, steps physics), so it must run alone. |
| `inline=True` | Reads only plain Python state and never touches Kit: answered at once on the socket thread. |

## Changing a tool without restarting Kit

The `reload_tools` tool with `module="my_tools"` imports the package afresh and registers its
tools again. It refuses while one of the package's calls is running. A submodule named exactly
`_state` is not re-imported, so keep live objects there (a connection, an articulation handle).
MCP clients see the change after `refresh_tools`.

## Your project's state as tools

Prefer a small tool over asking an agent to reach into your modules with `run_python`: the
arguments are typed, the rules live in one place, and the tool still works when `run_python` is
refused (read-only mode). The template's `project_state` and
`set_threshold` show the shape.

## A server for one project

One bridge can serve several MCP servers. Tools of uncatalogued project extensions are part of the `sim` toolset, so a server
that lists them also lists the scene tools and `run_python`:

```json
{ "mcpServers": { "robot_demo": { "command": "twinsmith",
    "env": { "TWINSMITH_TOOLSETS": "default,+sim" } } } }
```

These settings choose what is listed. To stop changes for every client, start Kit read-only.

## Files on the simulator's machine

Tools run where Kit runs, so their code and assets must be on that machine. `put_file` writes a
file there, `stat_file` reports its size and SHA-256, and `get_file` reads one back. A file
travels in one message, so use them for code and small assets. `mcp/templates/remote_driver.py`
copies a package by hash and reloads it.

## Things Kit will surprise you with

- Kit takes `--enable <extension>` with a space. `--enable=<extension>` is ignored without an error.
- Inside a tool, `await omni.kit.app.get_app().next_update_async()` lets a frame pass. A tool that
  does this must be `exclusive`, or other calls could start between its frames.
- Opening a stage invalidates every prim and physics handle you hold. Fetch them again afterwards.
- With a window, the timeline keeps playing while your tool is not running. Headless under a
  script, nothing advances unless something updates the app.
- Do not call `World.step()` or `app.update()` from a tool on a playing timeline: you would step
  physics from inside a step. Await a frame instead, or use `sim.simulation.step`.
