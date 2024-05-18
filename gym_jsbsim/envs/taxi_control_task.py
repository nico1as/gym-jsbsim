from gym_jsbsim.task import Task
from gym_jsbsim.catalogs.catalog import Catalog as c
import random
import math

"""
    A task in which the agent must follow taxiway centerline trajectory.
    The aircraft velocity should not exceed 20 knot/s in straightline and 7 knots/s during turn.
"""


class TaxiControlTask(Task):
    state_var = [c.velocities_vc_fps, c.shortest_dist, c.d1, c.d2, c.d3, c.d4, c.a1, c.a2, c.a3, c.a4]

    action_var = [c.fcs_steer_cmd_norm, c.fcs_center_brake_cmd_norm, c.fcs_throttle_cmd_norm]

    k2f = 1.68781
    # INIT_AC_LON = 1.369889125000043  # loop
    # INIT_AC_LAT = 43.625578879000045 # loop
    INIT_AC_LON = 1.37211666700005708  # attol
    INIT_AC_LAT = 43.6189638890000424  # attol
    INIT_AC_HEADING = 323
    INITIAL_ALTITUDE_FT = 8.42
    INITIAL_VELOCITY_U = 7.0 * k2f  # 33.76 #20 knots/sec

    avg_dist = 0
    nb_step = 0
    perf_time = 0
    perf_time_avg = 0

    init_conditions = {
        c.ic_h_sl_ft: INITIAL_ALTITUDE_FT,
        c.ic_terrain_elevation_ft: INITIAL_ALTITUDE_FT,  # Blagnac Ariport Altittude (148.72m = 487.9265f)
        c.ic_h_agl_ft: INITIAL_ALTITUDE_FT,
        c.ic_long_gc_deg: INIT_AC_LON,
        c.ic_lat_geod_deg: INIT_AC_LAT,
        c.ic_u_fps: INITIAL_VELOCITY_U,  # ie: 10 knots
        c.ic_v_fps: 0,
        c.ic_w_fps: 0,
        c.ic_p_rad_sec: 0,
        c.ic_q_rad_sec: 0,
        c.ic_r_rad_sec: 0,
        c.ic_roc_fpm: 0,
        c.ic_psi_true_deg: INIT_AC_HEADING,
        c.target_heading_deg: INIT_AC_HEADING,
        c.fcs_throttle_cmd_norm: 0,
        c.fcs_mixture_cmd_norm: 1,
        c.gear_gear_pos_norm: 1,
        c.gear_gear_cmd_norm: 1,
        c.id_path: 0,
    }

    def get_reward(self, state, sim):
        """
        Reward with distance to the centerline and average velocity during the simulation
        """

        dist_r = math.exp(
            -sim.get_property_value(c.shortest_dist)
        )  # 1.0/math.sqrt((sim.get_property_value(c.shortest_dist)+1))

        self.avg_dist += math.fabs(sim.get_property_value(c.shortest_dist))
        self.nb_step += 1
        # average velocity with simulation time ponderation ... normalised

        self.perf_time += math.fabs(sim.get_property_value(c.velocities_vc_fps))
        av_vel = self.perf_time / self.nb_step
        dist_total = av_vel * sim.get_property_value(c.simulation_sim_time_sec)

        dist_total_norm = dist_total / (20.0 * self.k2f * sim.get_property_value(c.simulation_sim_time_sec))

        reward = 0.8 * dist_r + 0.2 * dist_total_norm

        # print(sim.get_property_value(c.shortest_dist), sim.get_property_value(c.velocities_vc_fps), sim.get_property_value(c.fcs_steer_cmd_norm))

        return reward

    def is_terminal(self, state, sim):

        terminal = (
            sim.get_property_value(c.simulation_sim_time_sec) >= 450
            or math.fabs(sim.get_property_value(c.shortest_dist)) >= 20
            or math.fabs(sim.get_property_value(c.velocities_vc_fps)) > 20 * self.k2f
            or math.fabs(sim.get_property_value(c.velocities_vc_fps)) < 5 * self.k2f
        )

        # if terminal:
        #     print('Simulation ended at t='+str(sim.get_property_value(c.simulation_sim_time_sec)))
        #     print('Avg dist to central line: '+str(self.avg_dist / self.nb_step))
        #     print('Avg vel: '+str(self.perf_time / self.nb_step))

        return terminal
