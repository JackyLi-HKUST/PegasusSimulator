#!/usr/bin/env python
"""
ArduPilot single-vehicle example WITHOUT ROS2 dependency.
Based on examples/11_ardupilot_multi_vehicle.py.
"""

import carb
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

# -----------------------------------
import omni.timeline
from omni.isaac.core.world import World

from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.backends.ardupilot_mavlink_backend import (
    ArduPilotMavlinkBackend, ArduPilotMavlinkBackendConfig)
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

from scipy.spatial.transform import Rotation


class PegasusApp:

    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()

        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])

        # Spawn ONE Iris drone with ArduPilot SITL auto-launched
        self.vehicle_factory(0, gap_x_axis=1.0)

        self.world.reset()
        self.stop_sim = False

    def vehicle_factory(self, vehicle_id: int, gap_x_axis: float):
        config_multirotor = MultirotorConfig()

        backend_config = ArduPilotMavlinkBackendConfig({
            "vehicle_id": vehicle_id,
            "ardupilot_autolaunch": True,
            "ardupilot_dir": self.pg.ardupilot_path,
            "ardupilot_vehicle_model": "gazebo-iris",
        })
        config_multirotor.backends = [
            ArduPilotMavlinkBackend(config=backend_config),
        ]

        Multirotor(
            f"/World/drone{vehicle_id}",
            ROBOTS['Iris'],
            vehicle_id,
            [gap_x_axis * vehicle_id, 0.0, 0.07],
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config_multirotor,
        )

    def run(self):
        self.timeline.play()
        while simulation_app.is_running() and not self.stop_sim:
            self.world.step(render=True)

        carb.log_warn("PegasusApp Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()


def main():
    pg_app = PegasusApp()
    pg_app.run()


if __name__ == "__main__":
    main()
