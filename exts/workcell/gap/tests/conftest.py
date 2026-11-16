"""Shared fixtures: load this registry once per session."""

from __future__ import annotations

from pathlib import Path

import pytest

#: This registry's checkout root (parent of tests/).
SKILLS_ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="session")
def skills_registry():
    from gap.skills import load_skills

    return load_skills(SKILLS_ROOT)


@pytest.fixture(scope="session")
def tool_registry(skills_registry):
    """ToolRegistry with the bundles' pending @tool registrations drained."""
    from gap_core.tools import ToolRegistry

    reg = ToolRegistry()
    reg.discover_pending()
    return reg
