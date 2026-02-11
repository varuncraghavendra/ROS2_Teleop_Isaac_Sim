#!/usr/bin/env python3
"""
Pegasus IRIS + PX4 MAVLink backend + plain USD Camera prim attached to the drone.

This is designed to avoid omni.syntheticdata / replicator completely,
because your Isaac Sim build is segfaulting inside libomni.syntheticdata.plugin.so.

What you get:
- Drone spawns + PX4 autolaunch
- Camera prim exists under the drone (viewable in Isaac UI)
- No LiDAR
- No Pegasus MonocularCamera
- No omni.isaac.sensor.Camera (often triggers syntheticdata pipeline)

Run:
  cd ~/isaacsim
  ./python.sh /home/varun/PegasusSimulator/examples/drone_px4_usd_camera_safe.py --px4_dir /path/to/PX4-Autopilot
"""

import argparse

def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--px4_dir", required=True, type=str, help="Path to PX4-Autopilot repo")
    p.add_argument("--headless", action="store_true", help="Run headless (no UI)")
    p.add_argument("--env", default="Warehouse with Shelves", type=str, help="Pegasus env name")
    return p.parse_args()

args = parse_args()

# IMPORTANT: create SimulationApp immediately after importing it
import carb
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": bool(args.headless)})

# ---- after SimulationApp ----
import omni
import omni.timeline
from omni.isaac.core.world import World
from omni.isaac.core.utils.prims import create_prim
from scipy.spatial.transform import Rotation

from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.backends.px4_mavlink_backend import (
    PX4MavlinkBackend,
    PX4MavlinkBackendConfig,
)

def disable_problem_extensions_best_effort():
    """
    Best-effort disable of extensions that commonly pull in SyntheticData/Replicator.
    If an extension is not present or cannot be disabled, we just warn and continue.
    """
    try:
        app = omni.kit.app.get_app()
        ext_mgr = app.get_extension_manager()

        # These are common culprits for SyntheticData graphs / replicator pipelines.
        to_disable = [
            "omni.syntheticdata",
            "omni.syntheticdata.scripts",
            "omni.replicator.core",
            "omni.replicator.isaac",
            "omni.replicator.agent",
        ]

        for ext in to_disable:
            try:
                if ext_mgr.is_extension_enabled(ext):
                    carb.log_warn(f"[SAFE MODE] Disabling extension: {ext}")
                    ext_mgr.set_extension_enabled_immediate(ext, False)
            except Exception as e:
                carb.log_warn(f"[SAFE MODE] Could not disable {ext}: {e}")

    except Exception as e:
        carb.log_warn(f"[SAFE MODE] Extension manager not available: {e}")

def attach_plain_usd_camera(drone_path: str) -> str:
    """
    Create a plain USD Camera prim under the drone.
    This does NOT use SyntheticData. It will show up in the stage and can be viewed from UI.
    """
    cam_path = f"{drone_path}/usd_camera"

    # Camera position relative to drone (forward + slight up)
    create_prim(
        prim_path=cam_path,
        prim_type="Camera",
        position=(0.20, 0.0, 0.12),
        orientation=(1.0, 0.0, 0.0, 0.0),  # identity quat (w,x,y,z)
    )

    # Optionally set some basic camera attributes via USD (safe to omit)
    try:
        import omni.usd
        from pxr import UsdGeom

        stage = omni.usd.get_context().get_stage()
        cam_prim = stage.GetPrimAtPath(cam_path)
        cam = UsdGeom.Camera(cam_prim)

        # Basic sane defaults
        cam.CreateFocalLengthAttr(18.0)
        cam.CreateHorizontalApertureAttr(20.955)  # mm-ish
        cam.CreateVerticalApertureAttr(15.2908)   # mm-ish
        cam.CreateClippingRangeAttr((0.05, 500.0))
    except Exception as e:
        carb.log_warn(f"[CAM] Could not set USD camera attrs (non-fatal): {e}")

    return cam_path

class App:
    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()

        # Disable SyntheticData/Replicator ASAP (best effort)
        disable_problem_extensions_best_effort()

        # Pegasus interface + world
        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Load environment
        env_key = args.env
        if env_key not in SIMULATION_ENVIRONMENTS:
            carb.log_warn(f"[ENV] Unknown env '{env_key}', falling back to 'Warehouse with Shelves'")
            env_key = "Warehouse with Shelves"
        self.pg.load_environment(SIMULATION_ENVIRONMENTS[env_key])

        # Multirotor config (NO graphical sensors!)
        cfg = MultirotorConfig()

        mav_cfg = PX4MavlinkBackendConfig(
            {
                "vehicle_id": 0,
                "px4_autolaunch": True,
                "px4_dir": args.px4_dir,
            }
        )
        cfg.backends = [PX4MavlinkBackend(mav_cfg)]
        cfg.graphical_sensors = []  # critical: no Pegasus MonocularCamera / Lidar

        self.drone_path = "/World/quadrotor"

        Multirotor(
            self.drone_path,
            ROBOTS["Iris"],
            0,
            [5.0, 0.0, 0.07],
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=cfg,
        )

        # Attach plain USD camera (safe)
        self.cam_path = attach_plain_usd_camera(self.drone_path)
        carb.log_info(f"[OK] Drone spawned: {self.drone_path}")
        carb.log_info(f"[OK] USD Camera attached: {self.cam_path}")

        self.world.reset()

    def run(self):
        self.timeline.play()
        while simulation_app.is_running():
            self.world.step(render=not args.headless)

        self.timeline.stop()
        simulation_app.close()

def main():
    App().run()

if __name__ == "__main__":
    main()

