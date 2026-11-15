"""Bridge handlers for the pack-backed workcell."""

FULL_REPORT = ("workcell__cell(action=\"status\") returns the full report: every part, pack file, camera intrinsic, "
               "Jenga block, the BOM audit and all limitations; workcell__robot, workcell__perception and "
               "workcell__jenga status return one section each.")


def summarize(report):
    """What a caller needs right after quick_start, in a few hundred tokens; the full report stays in status.
    Plain Python on the report dict, so it never fails where the build succeeded."""
    if not isinstance(report, dict) or report.get("state") != "ready":
        return report
    keep = ("state", "mode", "backend", "timeline", "gravity_m_s2", "robot_gravity_disabled", "tabletop_z_m")
    summary = {key: report[key] for key in keep if key in report}
    pack = report.get("pack") or {}
    summary["pack"] = {key: pack[key] for key in ("id", "version", "qualification") if key in pack}
    robot = report.get("robot") or {}
    summary["robot"] = {key: robot[key] for key in ("assembly_path", "robot_path", "articulation_root", "tool0_path",
                                                    "gripper_path", "max_opening_m") if key in robot}
    cameras = report.get("cameras") or {}
    if isinstance(cameras, dict):
        summary["cameras"] = {name: {key: camera.get(key) for key in ("status", "prim_path")}
                              for name, camera in cameras.items() if isinstance(camera, dict)}
    summary["task"] = (report.get("task") or {}).get("name")
    if "jenga" in report:
        summary["jenga"] = {key: report["jenga"][key] for key in ("block_count", "layers", "variation_profile", "variation_seed")
                            if key in report["jenga"]}
    summary["parts"] = len(report.get("parts") or [])
    audit = report.get("bom_audit") or {}
    summary["bom_audit"] = {"ok": audit.get("ok"), "errors": (audit.get("errors") or [])[:5]}
    newton = report.get("newton")
    if isinstance(newton, dict):
        buffers = newton.get("buffers") or {}
        summary["newton"] = {"steps_per_second": (newton.get("settings") or {}).get("newton:timeStepsPerSecond"),
                             **{key: buffers[key] for key in ("rigid_bodies", "nconmax", "njmax", "warnings")
                                if key in buffers}}
    for key in ("bench_top", "environment", "contact"):
        if key in report:
            summary[key] = report[key]
    limitations = report.get("limitations") or []
    summary["limitations"] = {"count": len(limitations),
                              "engine": [line for line in limitations if str(line).startswith("Newton:")]}
    summary["full_report"] = FULL_REPORT
    return summary


class Commands:
    async def _cmd_workcell_quick_start(self, config_path=None, mode="physics", discard_changes=False,
                                        environment="none", task=None):
        report = await self.workcell.quick_start(config_path=config_path, mode=mode, discard_changes=discard_changes,
                                                 environment=environment, task=task)
        return summarize(report)

    def _cmd_workcell_status(self):
        return self.workcell.status()

    def _cmd_workcell_shutdown(self):
        return self.workcell.shutdown()

    def _cmd_robot_status(self):
        report = self.workcell.status()
        return {"state": report.get("state"), "robot": report.get("robot")}

    def _cmd_robot_wrench(self):
        return self.workcell.read_wrench()

    def _cmd_robot_rail(self, position_mm=None):
        return self.workcell.rail(position_mm=position_mm)

    def _cmd_perception_status(self):
        report = self.workcell.status()
        return {"state": report.get("state"), "cameras": report.get("cameras")}

    def _cmd_jenga_status(self):
        report = self.workcell.status()
        return {"state": report.get("state"), "jenga": report.get("jenga")}

    def _cmd_jenga_build(self, profile=None, seed=None):
        return self.workcell.build_jenga(profile=profile, seed=seed)
