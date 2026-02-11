#!/usr/bin/env python3
"""
warehouse_multi_px4.py
- Loads Warehouse environment (Pegasus)
- Spawns N PX4 Iris drones in a line
  - default: along +Y (90° from X-line)
- Autolaunches PX4 instances (vehicle_id = 0..N-1)
- Disables Isaac ROS2 bridge extensions (not needed)

Run:
  ~/isaacsim/python.sh /home/varun/PegasusSimulator/examples/warehouse_multi_px4.py \
      --px4_dir /home/varun/PX4-Autopilot --num_drones 5 --spacing 2.0

To line along X instead:
  --line_axis x

To flip direction:
  --line_dir -
"""

import argparse
import os
import math


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--px4_dir", required=True, type=str)
    p.add_argument("--num_drones", type=int, default=3)
    p.add_argument("--env", type=str, default="Warehouse with Shelves")
    p.add_argument("--headless", action="store_true")
    p.add_argument("--spacing", type=float, default=2.0)
    p.add_argument("--start_x", type=float, default=4.0)
    p.add_argument("--start_y", type=float, default=0.0)
    p.add_argument("--start_z", type=float, default=0.07)
    p.add_argument("--px4_vehicle_model", type=str, default=None)

    # ✅ line arrangement controls
    p.add_argument("--line_axis", choices=["x", "y"], default="y",
                   help="Axis along which drones are placed in a line. y = 90° from x.")
    p.add_argument("--line_dir", choices=["+", "-"], default="+",
                   help="Direction along line axis.")
    return p.parse_args()


ARGS = parse_args()
if not os.path.isdir(ARGS.px4_dir):
    raise RuntimeError(f"PX4 dir not found: {ARGS.px4_dir}")

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": bool(ARGS.headless)})

import carb
import omni
import omni.timeline
from omni.isaac.core.world import World

from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig


def disable_ros2_bridge_best_effort():
    try:
        app = omni.kit.app.get_app()
        ext_mgr = app.get_extension_manager()
        for ext in ["isaacsim.ros2.bridge", "omni.isaac.ros2_bridge"]:
            try:
                if ext_mgr.is_extension_enabled(ext):
                    carb.log_warn(f"[EXT] Disabling {ext}")
                    ext_mgr.set_extension_enabled_immediate(ext, False)
            except Exception:
                pass
    except Exception:
        pass


def euler_to_quat_xyzw(roll, pitch, yaw):
    cr = math.cos(roll * 0.5); sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return [x, y, z, w]


class WarehouseMultiPX4:
    def __init__(self):
        disable_ros2_bridge_best_effort()
        self.timeline = omni.timeline.get_timeline_interface()

        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        env_name = ARGS.env
        if env_name not in SIMULATION_ENVIRONMENTS:
            carb.log_warn(f"[ENV] '{env_name}' not found. Using 'Warehouse with Shelves'.")
            env_name = "Warehouse with Shelves"

        self.pg.load_environment(SIMULATION_ENVIRONMENTS[env_name])
        carb.log_info(f"[ENV] Loaded: {env_name}")

        # ✅ Line arrangement: default along Y (+Y). This is the “90° line” from X.
        for i in range(ARGS.num_drones):
            self.spawn(i)

        self.world.reset()

    def _line_offset(self, i: int) -> float:
        sgn = 1.0 if ARGS.line_dir == "+" else -1.0
        return sgn * float(i) * float(ARGS.spacing)

    def spawn(self, vehicle_id: int):
        prim_path = f"/World/quadrotor_{vehicle_id}"

        # Start position
        x = float(ARGS.start_x)
        y = float(ARGS.start_y)
        z = float(ARGS.start_z)

        # Apply line offset along chosen axis
        off = self._line_offset(vehicle_id)
        if ARGS.line_axis == "x":
            x += off
        else:  # "y"
            y += off

        cfg = MultirotorConfig()
        px4_model = ARGS.px4_vehicle_model if ARGS.px4_vehicle_model else self.pg.px4_default_airframe

        mav_cfg = PX4MavlinkBackendConfig({
            "vehicle_id": vehicle_id,
            "px4_autolaunch": True,
            "px4_dir": ARGS.px4_dir,
            "px4_vehicle_model": px4_model,
        })

        cfg.backends = [PX4MavlinkBackend(mav_cfg)]
        cfg.graphical_sensors = []

        Multirotor(
            prim_path,
            ROBOTS["Iris"],
            vehicle_id,
            [x, y, z],
            euler_to_quat_xyzw(0.0, 0.0, 0.0),
            config=cfg,
        )

        carb.log_info(
            f"[SPAWN] id={vehicle_id} prim='{prim_path}' pos=({x:.2f},{y:.2f},{z:.2f}) "
            f"line_axis={ARGS.line_axis} line_dir={ARGS.line_dir} model='{px4_model}'"
        )

    def run(self):
        self.timeline.play()
        while simulation_app.is_running():
            self.world.step(render=not ARGS.headless)
        self.timeline.stop()
        simulation_app.close()


def main():
    WarehouseMultiPX4().run()


if __name__ == "__main__":
    main()

