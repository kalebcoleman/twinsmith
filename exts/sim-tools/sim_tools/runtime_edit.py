"""Release a physics edit-target lease after an externally requested Stop."""

import asyncio


def stop_edit_target_guard(scene, stage):
    """Accept our pending lease restoration, not another editor's target."""
    expected = stage.GetEditTarget()
    previous = scene._runtime_edit_target

    def validate():
        nonlocal expected, previous
        current = stage.GetEditTarget()
        if previous is not None and scene._runtime_edit_target is None and current == previous:
            expected, previous = previous, None
        if current != expected:
            from twinsmith.project_assets import refuse

            refuse("EDIT_TARGET_CHANGED", "Selected edit target changed during native Stop completion; the external target was preserved.")

    return validate


class RuntimeEditRestorer:
    def __init__(self, scene, next_update, report_error):
        self.scene = scene
        self.next_update = next_update
        self.report_error = report_error
        self.task = None

    def cancel(self):
        if self.task is not None:
            self.task.cancel()
            self.task = None

    def stopped(self):
        self.cancel()
        scene = self.scene
        stage = scene._stage
        lease = scene._runtime_edit_target
        if stage is None or lease is None:
            return
        self.task = asyncio.ensure_future(self._restore(stage, lease))

    async def _restore(self, stage, lease):
        try:
            # PhysX must finish its Stop/reset writeback before changing the
            # target. Observe the ordinary frame; never re-enter Kit's update.
            await asyncio.wait_for(self.next_update(), 5)
            scene = self.scene
            if (
                scene._stage == stage
                and scene.get_stage() == stage
                and scene._runtime_edit_target is lease
                and scene.timeline.is_stopped()
            ):
                scene.end_runtime_edit(stage)
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            self.report_error(
                "Could not restore the physics edit target after Stop; inspect "
                "the selected layer and retry sim.timeline(stop): " + str(exc)
            )
