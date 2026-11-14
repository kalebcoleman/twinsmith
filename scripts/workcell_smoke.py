"""Validate stopped-stage authoring/BOM; dynamics and renders need separate qualification."""

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "mcp"))
from twinsmith.client import BridgeClient  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, required=True)
    parser.add_argument("--config", required=True, help="Absolute config path on the Kit host")
    parser.add_argument("--mode", choices=("physics", "mirror_real"), default="physics")
    parser.add_argument("--mutate-stage", action="store_true", required=True)
    parser.add_argument("--discard-changes", action="store_true")
    args = parser.parse_args()
    if not Path(args.config).is_absolute():
        parser.error("--config must be absolute on the Kit host")
    with BridgeClient(host=args.host, port=args.port, timeout=120) as bridge:
        identity = bridge.call("status", {})
        print("Bridge:", json.dumps(identity, default=str), flush=True)
        code = f'''import omni.usd
from workcell.scene import Workcell
from pxr import UsdPhysics, UsdGeom, PhysxSchema
workcell = Workcell()
report = await workcell.quick_start(config_path={args.config!r}, mode={args.mode!r}, discard_changes={args.discard_changes!r},
                                    task="jenga_tower")
stage = omni.usd.get_context().get_stage()
assert report["state"] == "ready" and report["backend"] in ("physx", "newton") and report["timeline"] == "stopped"
if report["backend"] == "newton":
    scene = stage.GetPrimAtPath("/World/PhysicsScene")
    assert scene.GetAttribute("newton:timeStepsPerSecond").Get() == report["newton"]["settings"]["newton:timeStepsPerSecond"]
    for name, drive in report["newton"]["drives"].items():
        joint = next(p for p in stage.Traverse() if p.GetName() == name and p.IsA(UsdPhysics.Joint))
        assert abs(joint.GetAttribute("newton:armature").Get() - drive["armature_kgm2"]) < 1e-6, name
assert report["tabletop_z_m"] == 0.0 and UsdGeom.GetStageMetersPerUnit(stage) == 1.0
gravity = UsdPhysics.Scene.Get(stage, "/World/PhysicsScene").GetGravityMagnitudeAttr().Get()
assert abs(gravity - 9.81) < 1e-5
assembly_path = stage.GetPrimAtPath(report["robot"]["assembly_path"]).GetPath()
robot_bodies = 0
for prim in stage.Traverse():
    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        is_robot = prim.GetPath().HasPrefix(assembly_path)
        robot_bodies += int(is_robot)
        disabled = bool(PhysxSchema.PhysxRigidBodyAPI(prim).GetDisableGravityAttr().Get())
        assert disabled == (is_robot and {args.mode == 'mirror_real'!r}), str(prim.GetPath())
assert robot_bodies > 0
assert report["jenga"]["block_size_m"] == [0.072, 0.024, 0.014]
assert report["jenga"]["block_count"] == len(report["jenga"]["parts"])
assert report["jenga"]["block_count"] == 3 * report["jenga"]["layers"]
for part in report["parts"]:
    prim = stage.GetPrimAtPath(part["prim_path"])
    assert prim.IsValid() and prim.GetAttribute("workcell:asset_id").Get() == part["asset_id"]
    assert prim.GetAttribute("workcell:bom_id").Get() == part["bom_id"]
result = report'''
        result = bridge.call("run_python", {"code": code}, timeout=120)
        print(json.dumps(result, indent=2, default=str))
        if result.get("result", {}).get("state") != "ready":
            raise RuntimeError("workcell smoke did not return a ready report")
        print(f"PASS ({result['result']['backend']}): stopped-stage authoring/BOM only; dynamics and captures are not tested.")


if __name__ == "__main__":
    main()
