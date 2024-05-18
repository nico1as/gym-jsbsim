from gym_jsbsim.task import Task
from gym_jsbsim.catalogs.catalog import Catalog as c
import math
import random
import numpy as np

"""
    A draft task to perform approach.
"""


class ApproachControlTask(Task):

    state_var = [c.delta_altitude, c.delta_heading, c.velocities_v_down_fps, c.velocities_vc_fps]

    action_var = [
        c.fcs_aileron_cmd_norm,
        c.fcs_elevator_cmd_norm,
        c.fcs_rudder_cmd_norm,
        c.fcs_throttle_cmd_norm,
    ]

    init_conditions = {
        c.ic_h_sl_ft: 2000,
        c.ic_terrain_elevation_ft: 0,
        c.ic_long_gc_deg: 1.493405,  # 1.442031
        c.ic_lat_geod_deg: 43.502138,  # 43.607181,
        c.ic_u_fps: 800,
        c.ic_v_fps: 0,
        c.ic_w_fps: 0,
        c.ic_p_rad_sec: 0,
        c.ic_q_rad_sec: 0,
        c.ic_r_rad_sec: 0,
        c.ic_roc_fpm: 0,
        c.ic_psi_true_deg: 323,
        c.target_heading_deg: 323,
        c.target_altitude_ft: 2000,
        c.fcs_throttle_cmd_norm: 0.8,
        c.fcs_mixture_cmd_norm: 1,
        c.gear_gear_pos_norm: 0,
        c.gear_gear_cmd_norm: 0,
        c.steady_flight: 150,
    }

    def get_reward(self, state, sim):

        heading_r = math.exp(-math.fabs(sim.get_property_value(c.delta_heading)))

        alt_r = math.exp(-math.fabs(sim.get_property_value(c.delta_altitude)))

        angle_speed_r = math.exp(
            -(
                0.1 * math.fabs(sim.get_property_value(c.accelerations_a_pilot_x_ft_sec2))
                + 0.1 * math.fabs(sim.get_property_value(c.accelerations_a_pilot_y_ft_sec2))
                + 0.8 * math.fabs(sim.get_property_value(c.accelerations_a_pilot_z_ft_sec2))
            )
        )

        reward = 0.4 * heading_r + 0.1 * alt_r + 0.4 * angle_speed_r

        return reward

    def is_terminal(self, state, sim):
        sim.set_property_value(c.target_altitude_ft, sim.get_property_value(c.target_altitude_ft) - 1)
        return (
            sim.get_property_value(c.target_altitude_ft) < 20
            or math.fabs(sim.get_property_value(c.delta_altitude)) > 5000
        )
