"""scripts/check_docs.py: tool names, retired phrases, generated counts, the work cell's own page.

Counts are never typed in a page, so there is nothing here to keep in step with a tool being added:
the generated blocks and the tool reference are rewritten by ``scripts/gen_tool_reference.py --write``.
"""

import asyncio
import importlib.util
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
_spec = importlib.util.spec_from_file_location("check_docs", ROOT / "scripts" / "check_docs.py")
check_docs = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(check_docs)
import gen_tool_reference  # noqa: E402


def page(tmp_path, text, name="page.md"):
    path = tmp_path / name
    path.write_text(text, encoding="utf-8")
    return path


def names_errors(tmp_path, text, **kwargs):
    errors = []
    check_docs.check_tool_names(errors, files=[page(tmp_path, text)], **kwargs)
    return errors


def test_the_repository_names_only_registered_tools():
    errors = []
    check_docs.check_tool_names(errors)
    assert errors == []


def test_a_registered_name_and_a_family_pass(tmp_path):
    assert names_errors(tmp_path, "Use `sim__prim__create`, `sim__prim` and `sim__stage__*`.") == []
    assert names_errors(tmp_path, "The `sim__camera__*` tools.") == []


@pytest.mark.parametrize("prefix", ["sim__", "assets__", "isaaclab__", "workcell__"])
def test_an_unregistered_name_is_reported_for_every_namespace(tmp_path, prefix):
    errors = names_errors(tmp_path, f"Call `{prefix}no_such_tool` first.")
    assert errors == [f"page.md: names a tool that is not registered: {prefix}no_such_tool"]


def test_a_family_must_prefix_a_real_tool(tmp_path):
    errors = names_errors(tmp_path, "Use `workcell__quickstart*`.")
    assert errors == ["page.md: names a tool that is not registered: workcell__quickstart*"]


def test_a_retired_bare_name_is_reported(tmp_path):
    errors = names_errors(tmp_path, "Run `list_bridges`, then screen_info.")
    assert sorted(errors) == ["page.md: names a retired tool: list_bridges",
                              "page.md: names a retired tool: screen_info"]
    # A longer word that merely contains a retired name is not a mention of it.
    assert names_errors(tmp_path, "my_list_bridges_helper") == []


def test_an_action_must_belong_to_its_group(tmp_path):
    assert names_errors(tmp_path, 'Use `windows(action="info")` and `jobs action=tail`.') == []
    errors = names_errors(tmp_path, 'Use `windows(action="bogus")`.')
    assert errors == ["page.md: windows has no action 'bogus'"]
    errors = names_errors(tmp_path, "Use sim__prim action=teleport.")
    assert errors == ["page.md: sim__prim has no action 'teleport'"]


def test_an_unrelated_word_before_action_is_not_a_group(tmp_path):
    assert names_errors(tmp_path, "The tool takes the `action=run` argument; call(action=1).") == []


@pytest.mark.parametrize("phrase", check_docs.RETIRED_PHRASES)
def test_a_retired_phrase_is_reported(tmp_path, phrase):
    errors = []
    check_docs.check_retired_phrases(errors, files=[page(tmp_path, f"Note: {phrase}. Use it.")])
    assert len(errors) == 1 and phrase in errors[0]


def test_no_page_in_the_repository_carries_a_retired_phrase():
    errors = []
    check_docs.check_retired_phrases(errors)
    assert errors == []


def test_the_generated_reference_and_blocks_are_current():
    errors = []
    check_docs.check_generated(errors)
    assert errors == []


def test_a_stale_block_is_reported_and_refreshed(tmp_path):
    block = "<!-- generated:profiles -->\n| not | the | table |\n<!-- /generated -->\n"
    path = page(tmp_path, "# Page\n\n" + block)
    output = ROOT / "docs" / "TOOLS-ARGUMENTS.md"
    errors = []
    check_docs.check_generated(errors, output=output, files=[path])
    assert errors == [f"{path} is stale: run python3 scripts/gen_tool_reference.py --write"]
    path.write_text(gen_tool_reference.refresh_blocks(path.read_text()))
    assert "| `default` |" in path.read_text()
    errors = []
    check_docs.check_generated(errors, output=output, files=[path])
    assert not any(str(path) in error for error in errors)


def test_a_stale_tool_reference_is_reported(tmp_path):
    output = tmp_path / "TOOLS-ARGUMENTS.md"
    errors = []
    check_docs.check_generated(errors, output=output, files=[])
    assert len(errors) == 1 and "TOOLS-ARGUMENTS.md is stale" in errors[0]
    output.write_text(gen_tool_reference.render())
    errors = []
    check_docs.check_generated(errors, output=output, files=[])
    assert errors == []


def test_the_reference_has_one_section_per_toolset_in_registry_order():
    from twinsmith import toolsets
    text = (ROOT / "docs" / "TOOLS-ARGUMENTS.md").read_text()
    headings = [line.split("`")[1] for line in text.splitlines() if line.startswith("## `")]
    expected = [name for name in toolsets.TOOLSETS if gen_tool_reference.listed_by_toolset()[name]]
    assert headings == expected


def test_listed_counts_match_the_server():
    """The generator reads the registry without the MCP SDK; the server must list the same tools."""
    from twinsmith import server, toolsets
    from twinsmith.config import select
    instance = server.build_server({"TWINSMITH_TOOLSETS": "all,+computer"})
    listed = {tool.name for tool in asyncio.run(instance.list_tools())}
    expected = {name for names in gen_tool_reference.listed_by_toolset().values() for name in names}
    assert listed == expected
    for profile, enabled in toolsets.PROFILES.items():
        shown = server.build_server({"TWINSMITH_TOOLSETS": profile})
        count = len(asyncio.run(shown.list_tools()))
        assert count == sum(len(gen_tool_reference.listed_by_toolset()[name]) for name in enabled), profile
    assert select("default") == toolsets.DEFAULT


def test_every_listed_work_cell_tool_is_named_in_the_work_cell_readme():
    errors = []
    check_docs.check_workcell_documented(errors)
    assert errors == []
    check_docs.check_workcell_documented(errors, readme="no tools here", listed=["workcell__jenga"])
    assert errors == [f"{check_docs.WORKCELL_README}: does not name the listed work-cell tool workcell__jenga"]


def test_vendoring_rejects_a_bundled_asset_claim():
    errors = []
    check_docs.check_vendoring_claims(errors)
    assert errors == []
    text = (ROOT / "docs/VENDORING.md").read_text().replace(
        "extension code only", "the extension and its 111 MB of assets")
    errors = []
    check_docs.check_vendoring_claims(errors, text=text)
    assert any("code-only" in error for error in errors)
    errors = []
    check_docs.check_vendoring_claims(errors, text="Vendored path only.\n")
    assert any("ships none" in error for error in errors)


@pytest.mark.parametrize("name,block", [("README.md", "toolsets-full"), ("AGENTS.md", "toolsets-compact"),
                                        ("docs/TOOLSETS.md", "toolsets-full"), ("docs/TOOLSETS.md", "profiles"),
                                        (".agents/skills/twinsmith/SKILL.md", "profiles")])
def test_the_pages_that_show_counts_carry_their_generated_block(name, block):
    """A page that lost its marker would stop being checked, so the markers themselves are checked."""
    text = (ROOT / name).read_text(encoding="utf-8")
    assert f"<!-- generated:{block} -->" in text and "<!-- /generated -->" in text
    assert gen_tool_reference.BLOCKS[block]() in text


def test_every_toolset_row_in_the_generated_table_matches_the_registry():
    from twinsmith import toolsets
    table = gen_tool_reference.render_toolsets_compact()
    listed = gen_tool_reference.listed_by_toolset()
    for name in toolsets.TOOLSETS:
        assert f"| `{name}` |" in table
    assert sum(len(tools) for tools in listed.values()) == len(
        {tool for tools in listed.values() for tool in tools})
