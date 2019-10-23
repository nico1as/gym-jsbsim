from gym_jsbsim.task import Task
from gym_jsbsim.catalogs.catalog import Catalog as c
import math
import random
import numpy as np

"""

    A task in which the agent must perform steady, level flight maintaining its

    initial heading.
"""

class HeadingControlTask(Task):

    state_var = [c.delta_altitude,
                 c.delta_heading,
                 c.attitude_pitch_rad,
                 c.attitude_roll_rad,
                 c.velocities_v_down_fps,
                 c.velocities_vc_fps,
                 c.velocities_p_rad_sec,
                 c.velocities_q_rad_sec,
                 c.velocities_r_rad_sec
                 ]

    action_var = [c.fcs_aileron_cmd_norm,
                  c.fcs_elevator_cmd_norm,
                  c.fcs_rudder_cmd_norm,
                  c.fcs_throttle_cmd_norm,
                  ]

    init_conditions = { # 'ic/h-sl-ft', 'initial altitude MSL [ft]'
                        c.ic_h_sl_ft: 10000,
                        #'ic/terrain-elevation-ft', 'initial terrain alt [ft]'
                        c.ic_terrain_elevation_ft: 0,
                        #'ic/long-gc-deg', 'initial geocentric longitude [deg]'
                        c.ic_long_gc_deg: 1.442031,
                        #'ic/lat-geod-deg', 'initial geodesic latitude [deg]'
                        c.ic_lat_geod_deg: 43.607181,
                        #'ic/u-fps', 'body frame x-axis velocity; positive forward [ft/s]'
                        c.ic_u_fps: 800,
                        #'ic/v-fps', 'body frame y-axis velocity; positive right [ft/s]'
                        c.ic_v_fps: 0,
                        #'ic/w-fps', 'body frame z-axis velocity; positive down [ft/s]'
                        c.ic_w_fps: 0,
                        #'ic/p-rad_sec', 'roll rate [rad/s]'
                        c.ic_p_rad_sec: 0,
                        #'ic/q-rad_sec', 'pitch rate [rad/s]'
                        c.ic_q_rad_sec: 0,
                        #'ic/r-rad_sec', 'yaw rate [rad/s]'
                        c.ic_r_rad_sec: 0,
                        #'ic/roc-fpm', 'initial rate of climb [ft/min]'
                        c.ic_roc_fpm: 0,
                        #'ic/psi-true-deg', 'initial (true) heading [deg]'
                        c.ic_psi_true_deg: 100,
                        # target heading deg
                        c.target_heading_deg: 100,
                        # target heading deg
                        c.target_altitude_ft: 10000,
                        # controls command
                        #'fcs/throttle-cmd-norm', 'throttle commanded position, normalised', 0., 1.
                        c.fcs_throttle_cmd_norm: 0.8,
                        #'fcs/mixture-cmd-norm', 'engine mixture setting, normalised', 0., 1.
                        c.fcs_mixture_cmd_norm: 1,
                        # gear up
                        c.gear_gear_pos_norm : 0,
                        c.gear_gear_cmd_norm: 0,
                        c.steady_flight:150
    }

    def get_reward(self, state, sim):
        '''
        Compute reward for HeadingControlTask
        '''
        # reward signal is built as a geometric mean of scaled gaussian rewards for each relevant variable

        heading_error_scale = 5. # degrees
        heading_r = math.exp(-(sim.get_property_value(c.delta_heading)/heading_error_scale)**2)

        alt_error_scale = 100. # degrees
        alt_r = math.exp(-(sim.get_property_value(c.delta_altitude)/alt_error_scale)**2)

        roll_error_scale = 0.09 # radians ~= 5 degrees
        roll_r = math.exp(-(sim.get_property_value(c.attitude_roll_rad)/roll_error_scale)**2)

        speed_error_scale = 16 # fps (~5%)
        speed_r = math.exp(-((sim.get_property_value(c.velocities_u_fps) - 800)/speed_error_scale)**2)

        # accel scale in "g"s
        accel_error_scale_x = 0.1
        accel_error_scale_y = 0.1
        accel_error_scale_z = 1.0
        accel_r = math.exp(-((sim.get_property_value(c.accelerations_n_pilot_x_norm)/accel_error_scale_x)**2 +
                             (sim.get_property_value(c.accelerations_n_pilot_y_norm)/accel_error_scale_y)**2 +
                             ((sim.get_property_value(c.accelerations_n_pilot_z_norm) + 1)/accel_error_scale_z)**2) #  normal value for z component is -1 g
                           )**(1/3) #  geometric mean

        reward = (heading_r * alt_r * accel_r * roll_r * speed_r)**(1/5) #  geometric mean
        return reward

    def is_terminal(self, state, sim):
        # Change heading every 150 seconds
        if sim.get_property_value(c.simulation_sim_time_sec) >= sim.get_property_value(c.steady_flight):
            # if the traget heading was not reach before, we stop the simulation
            if math.fabs(sim.get_property_value(c.delta_heading)) > 10:
                return True
            if math.fabs(sim.get_property_value(c.delta_altitude)) >= 100:
                return True

            alt_delta = int(sim.get_property_value(c.steady_flight)/150) * 100
            sign = random.choice([+1., -1.])
            new_alt = sim.get_property_value(c.target_altitude_ft) + sign * alt_delta

            angle = int(sim.get_property_value(c.steady_flight)/150) * 10
            sign = random.choice([+1., -1.])
            new_heading = sim.get_property_value(c.target_heading_deg) + sign * angle
            new_heading = (new_heading +360) % 360

            print(f'Time to change: {sim.get_property_value(c.simulation_sim_time_sec)} (Altitude: {sim.get_property_value(c.target_altitude_ft)} -> {new_alt}, Heading: {sim.get_property_value(c.target_heading_deg)} -> {new_heading})')
            sim.set_property_value(c.target_altitude_ft, new_alt)
            sim.set_property_value(c.target_heading_deg, new_heading)

            sim.set_property_value(c.steady_flight,sim.get_property_value(c.steady_flight)+150)
        # End up the simulation if the aircraft is on an extreme state
        # TODO: Why is an altitude check needed?
        return (sim.get_property_value(c.position_h_sl_ft) < 3000) or bool(sim.get_property_value(c.detect_extreme_state))
