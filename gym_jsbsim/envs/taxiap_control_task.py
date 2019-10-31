from gym_jsbsim.task import Task
from gym_jsbsim.catalogs.catalog import Catalog as c
import random
import math

"""
    A task in which the agent must follow taxiway centerline trajectory.
    The aircraft velocity should not exceed 20 knot/s in straightline and 7 knots/s during turn.
"""

class TaxiapControlTask(Task):
    state_var = [c.velocities_vc_fps, c.shortest_dist, c.d1, c.d2, c.d3, c.d4, c.a1, c.a2, c.a3, c.a4]
    
    #state_var = [c.velocities_vc_fps, c.delta_heading]#c.shortest_dist, c.a1, c.d1]

    action_var = [c.fcs_steer_cmd_norm]

    k2f = 1.68781
    INIT_AC_LON = 1.37211666700005708
    INIT_AC_LAT = 43.6189638890000424
    INIT_AC_HEADING = 323
    INITIAL_ALTITUDE_FT = 11.52
    INITIAL_VELOCITY_U = 7.0 * k2f #33.76 #20 knots/sec

    avg_dist = 0
    nb_step = 0

    init_conditions = { # 'ic/h-sl-ft', 'initial altitude MSL [ft]'
                        c.ic_h_sl_ft: INITIAL_ALTITUDE_FT,
                        #'ic/terrain-elevation-ft', 'initial terrain alt [ft]'
                        c.ic_terrain_elevation_ft: INITIAL_ALTITUDE_FT, # Blagnac Ariport Altittude (148.72m = 487.9265f)
                        c.ic_h_agl_ft : INITIAL_ALTITUDE_FT,
                        #'ic/long-gc-deg', 'initial geocentric longitude [deg]'
                        c.ic_long_gc_deg: INIT_AC_LON,
                        #'ic/lat-geod-deg', 'initial geodesic latitude [deg]'
                        c.ic_lat_geod_deg: INIT_AC_LAT,
                        #'ic/u-fps', 'body frame x-axis velocity; positive forward [ft/s]'
                        c.ic_u_fps: INITIAL_VELOCITY_U, # ie: 10 knots
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
                        c.ic_psi_true_deg: INIT_AC_HEADING,
                        # target heading deg
                        c.target_heading_deg: INIT_AC_HEADING,
                        #'fcs/throttle-cmd-norm', 'throttle commanded position, normalised', 0., 1.
                        c.fcs_throttle_cmd_norm: 0,
                        #'fcs/mixture-cmd-norm', 'engine mixture setting, normalised', 0., 1.
                        c.fcs_mixture_cmd_norm: 1,
                        # all engine running
                        c.propulsion_set_running: -1,
                        # gear up
                        c.gear_gear_pos_norm : 1,
                        c.gear_gear_cmd_norm: 1,
                        # AP ON
                        c.ap_vg_hold: 1,
                        # id_path
                        c.id_path : 0
    }


    def get_reward(self,state, sim):
        '''
        Reward according to distance to the centerline.
        '''
        shortest_dist_r = math.exp(-math.fabs(sim.get_property_value(c.shortest_dist)))
        
        self.avg_dist += math.fabs(sim.get_property_value(c.shortest_dist))
        self.nb_step += 1

        print(sim.get_property_value(c.shortest_dist), sim.get_property_value(c.velocities_vc_fps), sim.get_property_value(c.fcs_steer_cmd_norm))
        
        
        return shortest_dist_r


    def is_terminal(self, state, sim):
        #print((sim.get_property_value(c.position_vrp_gc_latitude_deg), sim.get_property_value(c.ic_lat_geod_deg) ,sim.get_property_value(c.position_vrp_longitude_deg), sim.get_property_value(c.ic_long_gc_deg)))
        # End up the simulation after 1200 secondes or if the aircraft is under or above 500 feet of its target altitude or velocity under 400f/s
        #print("state", state)
        #print("sim.get_property_value(v_air)", sim.get_property_value(v_air), sim.get_property_value(u_fps), sim.get_property_value(v_fps))
        
        '''
        LAWS 1 (static): velocity < 7 knots in turn and < 20 knots in straight line
        '''
        # TURN
        a1 = sim.get_property_value(c.a1)
        if abs(a1) > 15:
            sim.set_property_value(c.target_vg, 7.0*self.k2f)
        else: # STRAIGHTLINE
            sim.set_property_value(c.target_vg, 20.0*self.k2f)

        
        #sim.set_property_value(c.target_heading_deg, (sim.get_property_value(c.attitude_psi_deg) + a1) % 360)


        # TOTEST
        # . commencer les reward qu'a partir de 30sec
        # . modifier le max vitesse avion pour le taxi
        # . reward de la distance parcouru tant que distance a la ligne central < a 20 m
        # . reward sur le nombre de points passé (ie: id_path)

        '''
        if sim.get_property_value(c.simulation_sim_time_sec) < 30:
            max_centerline_distance = 40
        elif sim.get_property_value(c.simulation_sim_time_sec) < 60:
            max_centerline_distance = 20
        elif sim.get_property_value(c.simulation_sim_time_sec) < 90:
            max_centerline_distance = 10
        elif sim.get_property_value(c.simulation_sim_time_sec) < 120:
            max_centerline_distance = 5
        else:
            max_centerline_distance = 1
        '''

        terminal = sim.get_property_value(c.simulation_sim_time_sec)>=150 or math.fabs(sim.get_property_value(c.shortest_dist)) >= 10
        if terminal:
            print('Simulation ended at t='+str(sim.get_property_value(c.simulation_sim_time_sec)))
            print('Avg dist to central line: '+str(self.avg_dist / self.nb_step))
        return terminal


