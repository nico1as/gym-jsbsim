from gym_jsbsim.task import Task
from gym_jsbsim.catalogs.catalog import Catalog as c
import random
import math

"""
    A task in which the agent must follow a taxiway centerline trajectory.
    The aircraft velocity should not exceed 20 knot/s in straightline and 7 knots/s during turn.
"""


class TaxiapControlTask(Task):
    # Define State and Action
    state_var = [c.velocities_vc_fps, c.shortest_dist, c.d1, c.d2, c.d3, c.d4, c.a1, c.a2, c.a3, c.a4]
    action_var = [c.fcs_steer_cmd_norm]

    k2f = 1.68781  # Knot to feet
    INIT_AC_LON = 1.369889125000043  # loop
    INIT_AC_LAT = 43.625578879000045  # loop
    # INIT_AC_LON = 1.37211666700005708 # attol
    # INIT_AC_LAT = 43.6189638890000424 # attol
    INIT_AC_HEADING = 323
    INITIAL_ALTITUDE_FT = 8.42
    INITIAL_VELOCITY_U = 7.0 * k2f  # 20 knots/sec

    avg_dist = 0
    nb_step = 0
    r = 1

    # Set Initial condition for Taxi with Auto Pilot
    init_conditions = {
        c.ic_h_sl_ft: INITIAL_ALTITUDE_FT,
        c.ic_terrain_elevation_ft: INITIAL_ALTITUDE_FT,
        c.ic_h_agl_ft: INITIAL_ALTITUDE_FT,
        c.ic_long_gc_deg: INIT_AC_LON,
        c.ic_lat_geod_deg: INIT_AC_LAT,
        c.ic_u_fps: INITIAL_VELOCITY_U,
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
        c.gear_gear_pos_norm: 1,  # Landing gear ON
        c.gear_gear_cmd_norm: 1,  # Landing gear ON
        c.ap_vg_hold: 1,  # AP ON
        c.id_path: 0,  # ID runaway path
    }

    def get_reward(self, state, sim):
        """
        Reward according to distance to the centerline.
        """
        shortest_dist_r = math.exp(-math.fabs(sim.get_property_value(c.shortest_dist)))

        self.avg_dist += math.fabs(sim.get_property_value(c.shortest_dist))
        self.nb_step += 1

        # print(sim.get_property_value(c.shortest_dist), sim.get_property_value(c.velocities_vc_fps), sim.get_property_value(c.fcs_steer_cmd_norm))

        return shortest_dist_r * shortest_dist_r

    def is_terminal(self, state, sim):

        """
        End the simulation if aircraft is up to 10 meters from the centerline
        """

        # Add random noise to velocity command
        # if (sim.get_property_value(c.id_path)%10>=1 and sim.get_property_value(c.id_path)%10<2):
        #     self.r = random.uniform(0.0, 1.5)

        # Set velocity of the aircraft according to turn and straight line
        a = sim.get_property_value(c.a3)
        if abs(a) > 10:  # TURN
            sim.set_property_value(c.target_vg, 7.0 * self.k2f * self.r)
        else:  # STRAIGHTLINE
            sim.set_property_value(c.target_vg, 15.0 * self.k2f * self.r)

        terminal = (
            sim.get_property_value(c.simulation_sim_time_sec) >= 450
            or math.fabs(sim.get_property_value(c.shortest_dist)) >= 10
            or math.fabs(sim.get_property_value(c.velocities_vc_fps)) <= 5.0 * self.k2f
        )

        # Some debug prints with average distance to the centerline
        # if terminal:
        #     print('Simulation ended at t='+str(sim.get_property_value(c.simulation_sim_time_sec)))
        #     print('Avg dist to central line: '+str(self.avg_dist / self.nb_step))
        #     print('shortest distance: '+str(sim.get_property_value(c.shortest_dist)))

        return terminal
