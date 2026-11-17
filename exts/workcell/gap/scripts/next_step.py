"""Route on what the scan says the build has to do next: the base's name, a piece's name, or ``done``."""

from typing import TypedDict


class Output(TypedDict):
    route: str


def run(ctx, *, next: str) -> Output:  # noqa: A002 - the scan's own word for it
    return {"route": next}
