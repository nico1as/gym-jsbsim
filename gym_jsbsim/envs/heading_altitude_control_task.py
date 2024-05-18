from gym_jsbsim.task import Task
from gym_jsbsim.envs.heading_control_task import HeadingControlTask
from gym_jsbsim.catalogs.catalog import Catalog as c
import math
import random
import numpy as np

"""
    A task in which the agent must perform steady, level flight following a certain heading and altitude.
    Every 150 sec a new target heading and altitude are set.
"""


class HeadingAltitudeControlTask(HeadingControlTask):
    def is_terminal(self, state, sim):
        # Change heading and altitude every 150 seconds
        if sim.get_property_value(c.simulation_sim_time_sec) >= sim.get_property_value(c.steady_flight):
            # If the target heading and altitude were not reached, we stop the simulation
            if math.fabs(sim.get_property_value(c.delta_heading)) > 10:
                return True
            if math.fabs(sim.get_property_value(c.delta_altitude)) >= 100:
                return True

            alt_delta = (int(sim.get_property_value(c.steady_flight) / 150) * 100) % 5000
            sign = random.choice([+1.0, -1.0])
            new_alt = sim.get_property_value(c.target_altitude_ft) + sign * alt_delta

            angle = int(sim.get_property_value(c.steady_flight) / 150) * 10
            sign = random.choice([+1.0, -1.0])
            new_heading = sim.get_property_value(c.target_heading_deg) + sign * angle
            new_heading = (new_heading + 360) % 360

            # print(f'Time to change: {sim.get_property_value(c.simulation_sim_time_sec)} (Altitude: {sim.get_property_value(c.target_altitude_ft)} -> {new_alt}, Heading: {sim.get_property_value(c.target_heading_deg)} -> {new_heading})')

            sim.set_property_value(c.target_altitude_ft, max(new_alt, 3000))
            sim.set_property_value(c.target_heading_deg, new_heading)

            sim.set_property_value(c.steady_flight, sim.get_property_value(c.steady_flight) + 150)

        # if acceleration are too high stop the simulation
        acceleration_limit_x = 2.0  # "g"s
        acceleration_limit_y = 2.0  # "g"s
        acceleration_limit_z = 2.0  # "g"s
        if sim.get_property_value(c.simulation_sim_time_sec) > 10:
            if (
                math.fabs(sim.get_property_value(c.accelerations_n_pilot_x_norm)) > acceleration_limit_x
                or math.fabs(sim.get_property_value(c.accelerations_n_pilot_y_norm)) > acceleration_limit_y
                or math.fabs(sim.get_property_value(c.accelerations_n_pilot_z_norm) + 1) > acceleration_limit_z
            ):  # z component is expected to be -1 g
                return True

        # End up the simulation if the aircraft is on an extreme state
        # TODO: Is an altitude check needed?
        return (sim.get_property_value(c.position_h_sl_ft) < 3000) or bool(
            sim.get_property_value(c.detect_extreme_state)
        )
