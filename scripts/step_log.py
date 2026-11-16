"""A log of a run's steps: what each was to do, when it ran in simulated and in wall time, where the wall time went,
and what came of it. Steps nest: one opened while another is open is its child.

``clock`` is a function of no arguments that returns the run's counters now: ``{"sim_s", "frame", "sample",
"spent"}``, where ``frame`` and ``sample`` count the film's frames and the joint samples so far (so a step knows
which of them are its own) and ``spent`` is ``{category: seconds or count}``, only ever growing. Stdlib only.
"""

import contextlib
import functools
import time


class StepLog:
    def __init__(self, clock):
        self.clock, self.steps, self.open = clock, [], []

    @contextlib.contextmanager
    def step(self, kind, piece=None, **plan):
        """Record a step around a block. The block gets the step's row and may set ``row["result"]``; an exception
        is recorded as ``row["error"]`` and passed on."""
        before = self.clock()
        row = {"id": len(self.steps), "kind": kind, "piece": piece, "parent": self.open[-1] if self.open else None, "plan": plan,
               "wall_start": time.time(), "sim_start": before["sim_s"], "frame_start": before["frame"], "sample_start": before["sample"]}
        self.steps.append(row)
        self.open.append(row["id"])
        try:
            yield row
        except Exception as error:
            row["error"] = str(error)
            raise
        finally:
            self.open.pop()
            after = self.clock()
            row.update(wall_s=round(time.time() - row["wall_start"], 3), sim_s=round(after["sim_s"] - before["sim_s"], 4),
                       frame_end=after["frame"], sample_end=after["sample"],
                       spent={key: round(value - before["spent"].get(key, 0), 4) for key, value in after["spent"].items()
                              if value != before["spent"].get(key, 0)})

    def note(self, **plan):
        """Add to the plan of the step that is open now."""
        self.steps[self.open[-1]]["plan"].update(plan)


def stepped(kind):
    """A method of an object with a ``log`` (a StepLog) as one step of that log: its ``piece`` argument names the
    step's piece, and what it returns is the step's result."""
    def wrap(method):
        @functools.wraps(method)
        def run(self, *args, **kwargs):
            with self.log.step(kind, kwargs.get("piece", args[0] if args else None)) as row:
                row["result"] = result = method(self, *args, **kwargs)
                return result
        return run
    return wrap
