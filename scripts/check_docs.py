"""Check the documentation against the code: links, JSON examples, tool names, retired phrases and counts.

Tool counts are never typed in a page. A page that shows one wraps it in a generated block, which
``scripts/gen_tool_reference.py --write`` fills from the registry; this check fails when a block or
the generated tool reference has drifted.
"""

import json
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "mcp"))
sys.path.insert(0, str(ROOT / "scripts"))

SKILLS_DIR = ROOT / ".agents" / "skills"

# Tools that no longer exist, by bare name. A namespaced name (sim__, assets__, isaaclab__, workcell__)
# needs no entry here: it must be registered, or it is reported.
RETIRED_TOOLS = frozenset({"list_bridges", "screen_info", "assets__prepare_workstation",
                           "isaaclab__run_script", "isaaclab__runs_cancel", "window_input", "window_click",
                           "window_drag", "window_scroll", "window_type", "window_key"})

# Sentences that were true once and are not any more: Newton is the stated target engine.
RETIRED_PHRASES = ("PhysX is the release target", "Newton is experimental")

# Toolsets folded into computer and sim. Selecting one (`+lab`) stops the server, so no page may tell a reader to.
RETIRED_TOOLSETS = re.compile(r"(?<![A-Za-z0-9_])\+(screen|input|launch|lab|ros2|workcell|extensions)(?![A-Za-z0-9_-])")
# Tools renamed without an alias. Only the rename table may name them.
RENAMES_PAGE = "docs/TOOLSETS.md"
RENAMED_TOOLS = frozenset({"sim__session", "sim__launch", "sim__list", "sim__status", "sim__stop", "sim__timeline__control",
                           "sim__viewport__frame", "sim__ros2__tf_setup", "sim__map__occupancy"})

# Generated from the tool descriptions in the code: a stale name there is fixed in the description, not in the page.
GENERATED_PAGES = frozenset({"docs/TOOLS-ARGUMENTS.md"})

NAMESPACED = re.compile(r"(?<![A-Za-z0-9_])((?:sim|assets|isaaclab|workcell)__[a-z][a-z0-9_]*)(\*?)")
# `windows(action="info")`, `windows action=info`, `jobs` with action `tail`, in code or prose.
GROUP_CALL = re.compile(r"(?<![A-Za-z0-9_])(?P<group>[a-z][a-z0-9_]*)(?:\(|\s+)action=[\"']?(?P<action>[a-z_0-9]+)")


def documentation_files():
    """Every Markdown page a reader or an agent is sent to."""
    return [ROOT / "README.md", ROOT / "AGENTS.md", *sorted((ROOT / "docs").glob("*.md")),
            ROOT / "mcp/README.md", *sorted((ROOT / "mcp/docs").glob("*.md")),
            *sorted((ROOT / "exts").glob("*/README.md")), *sorted((ROOT / "exts").glob("*/docs/*.md")),
            *sorted((ROOT / "exts").glob("*/workcell/*.md")), *sorted(SKILLS_DIR.rglob("*.md"))]


def registered():
    """Every tool name the code registers, and every grouped tool's listed name."""
    from gen_tool_reference import registered_names
    return registered_names()


def check_tool_names(errors, files=None, names=None, groups=None, retired=None):
    """A page may name only tools that exist: namespaced names must be registered, a retired bare
    name is refused, and the action in `group(action="x")` must belong to that group."""
    from twinsmith import tool_groups

    names = registered() if names is None else names
    groups = {group: set(actions) for group, (_, actions) in tool_groups.GROUPS.items()} if groups is None else groups
    retired = RETIRED_TOOLS if retired is None else retired
    for path in files if files is not None else documentation_files():
        text = path.read_text(encoding="utf-8")
        try:
            where = path.relative_to(ROOT)
        except ValueError:
            where = path.name
        if str(where) in GENERATED_PAGES:
            continue
        seen = set()
        for name, star in NAMESPACED.findall(text):
            if name in names and not star:
                continue
            # A trailing `*` marks a family (`workcell__quick_start*`): it must prefix a real tool.
            if star and any(known.startswith(name) for known in names):
                continue
            seen.add(name + star)
        if str(where) == RENAMES_PAGE:
            seen -= RENAMED_TOOLS
        for name in sorted(seen):
            errors.append(f"{where}: names a tool that is not registered: {name}")
        for name in sorted(retired):
            if re.search(r"(?<![A-Za-z0-9_])" + re.escape(name) + r"(?![A-Za-z0-9_])", text):
                errors.append(f"{where}: names a retired tool: {name}")
        for match in GROUP_CALL.finditer(text):
            group, action = match.group("group"), match.group("action")
            if group in groups and action not in groups[group] | {"help"}:
                errors.append(f"{where}: {group} has no action {action!r}")


def check_retired_phrases(errors, files=None):
    for path in files if files is not None else documentation_files():
        text = path.read_text(encoding="utf-8")
        for phrase in RETIRED_PHRASES:
            if phrase in text:
                try:
                    where = path.relative_to(ROOT)
                except ValueError:
                    where = path.name
                errors.append(f"{where}: contains the retired phrase {phrase!r}; "
                              "the target engine is stated once, in docs/COMPATIBILITY.md")
        for match in RETIRED_TOOLSETS.finditer(text):
            try:
                where = path.relative_to(ROOT)
            except ValueError:
                where = path.name
            errors.append(f"{where}: selects the removed toolset {match.group(1)!r}; see {RENAMES_PAGE}#renamed-toolsets")


def check_generated(errors, output=None, files=None):
    """docs/TOOLS-ARGUMENTS.md and every generated count block must equal what the code produces."""
    from gen_tool_reference import stale_pages
    for page in stale_pages(output, files):
        errors.append(f"{page} is stale: run python3 scripts/gen_tool_reference.py --write")


# The work cell documents itself: its own README names every listed work-cell tool.
WORKCELL_README = "exts/workcell/workcell/README.md"


def check_workcell_documented(errors, readme=None, listed=None):
    """Every listed work-cell tool is named in the work cell's own README (the toolset's page)."""
    from gen_tool_reference import listed_by_toolset
    text = (ROOT / WORKCELL_README).read_text(encoding="utf-8") if readme is None else readme
    listed = [name for name in listed_by_toolset()["sim"] if name.startswith("workcell__")] if listed is None else listed
    for name in listed:
        if f"`{name}`" not in text:
            errors.append(f"{WORKCELL_README}: does not name the listed work-cell tool {name}")


def check_vendoring_claims(errors, text=None):
    """The vendoring page must not claim the UR5e asset payload is in the git tree."""
    text = (ROOT / "docs/VENDORING.md").read_text(encoding="utf-8") if text is None else text
    if re.search(r"\d+\s*MB of assets", text):
        errors.append("docs/VENDORING.md: claims a vendored asset payload; the tree is code-only")
    if "ships none" not in text:
        errors.append("docs/VENDORING.md: must state that the repository ships none of those assets")


def check_links_and_json(errors, files):
    for path in files:
        text = path.read_text(encoding="utf-8")
        for link in re.findall(r"\]\(([^)]+)\)", text):
            target = link.split("#", 1)[0]
            if not target or "://" in target or target.startswith("mailto:"):
                continue
            if not (path.parent / target).exists():
                errors.append(f"{path.relative_to(ROOT)}: missing local link {target}")
        for block in re.findall(r"```json[ \t]*\n(.*?)\n```", text, re.DOTALL):
            try:
                json.loads(block)
            except json.JSONDecodeError as exc:
                errors.append(f"{path.relative_to(ROOT)}: invalid JSON example: {exc}")


def main():
    from check_private_paths import check_tracked

    errors = check_tracked(ROOT)
    files = documentation_files()
    check_links_and_json(errors, files)
    check_tool_names(errors, files)
    check_retired_phrases(errors, files)
    check_workcell_documented(errors)
    check_vendoring_claims(errors)
    check_generated(errors)
    for error in errors:
        print(f"FAIL: {error}")
    if errors:
        return 1
    print(f"Documentation checks passed ({len(files)} files; links, JSON, registered tool names, retired "
          "phrases, generated tool reference and count blocks).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
