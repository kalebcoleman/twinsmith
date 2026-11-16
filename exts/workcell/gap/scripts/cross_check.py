"""Compare a vision-language model's count of the parts on the table with the scan's."""

import re
from typing import TypedDict


class Output(TypedDict):
    vlm_count: int
    scan_count: int
    vlm_agrees: bool
    answer: str


def run(ctx, *, answer: str, found: int) -> Output:
    """``answer`` is the model's reply to "how many parts"; its first whole number is the count (-1 when it
    gave none). ``found`` is how many pieces the scan named."""
    number = re.search(r"\d+", answer or "")
    count = int(number.group()) if number else -1
    return {"vlm_count": count, "scan_count": found, "vlm_agrees": count == found, "answer": (answer or "").strip()}
