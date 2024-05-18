import unittest
import math
import gym_jsbsim
from gym_jsbsim import Catalog as c


class TestSimulation(unittest.TestCase):
    """

    Class to test Simulation class and especially the get_state/set_state methods

    with different scenarios of actions.

    """

    # the properties to check at the end of the two simulations
    state_properties = [
        c.position_h_sl_ft,
        c.position_lat_geod_deg,
        c.position_long_gc_deg,
        c.velocities_u_fps,
        c.velocities_v_fps,
        c.velocities_w_fps,
        c.attitude_psi_deg,
        c.attitude_theta_deg,
        c.attitude_phi_deg,
        c.velocities_p_rad_sec,
        c.velocities_q_rad_sec,
        c.velocities_r_rad_sec,
    ]

    error_max = 0.0005

    tmax = 3600  # in seconds

    def setUp(self):
        self.env = gym_jsbsim.make("GymJsbsim-HeadingControlTask-v0")

    def tearDown(self):
        self.env.close()

    def test_get_state_set_state_no_action(self):

        self.env.reset()

        get_state = False
        count_actions = 0
        state = None
        t = 0

        # first flight
        while t <= self.tmax and self.env.sim.get_property_value(c.position_h_sl_ft) >= 200:
            if t >= 5 and get_state is False:
                # get_state after 5 seconds of simulation
                state = self.env.get_state()
                get_state = True
            if get_state:
                count_actions += 1
            self.env.step()
            t = self.env.get_sim_time()
        end_state_1 = self.env.get_state()

        # second flight
        self.env.set_state(state)
        for _ in range(count_actions):
            self.env.step()
        end_state_2 = self.env.get_state()

        # compute error between two end states
        for prop in self.state_properties:
            p1 = end_state_1[prop]
            p2 = end_state_2[prop]
            if p1 == p2:
                error = 0
            else:
                error = math.fabs(p2 - p1) / max(math.fabs(p1), math.fabs(p2))
            self.assertLess(error, self.error_max, "The two simulations have diverged")

    def test_get_state_set_state_constant_action(self):
        constant_action = [1, 1, 1, 1]

        self.env.reset()

        get_state = False
        count_actions = 0
        state = None
        t = 0

        # first flight
        while t <= self.tmax and self.env.sim.get_property_value(c.position_h_sl_ft) >= 200:
            if t >= 5 and get_state is False:
                # get_state after 5 seconds of simulation
                state = self.env.get_state()
                get_state = True
            if get_state:
                count_actions += 1
            self.env.step(constant_action)
            t = self.env.get_sim_time()
        end_state_1 = self.env.get_state()

        # second flight
        self.env.set_state(state)
        for _ in range(count_actions):
            self.env.step(constant_action)
        end_state_2 = self.env.get_state()

        # compute error between two end states
        for prop in self.state_properties:
            p1 = end_state_1[prop]
            p2 = end_state_2[prop]
            if p1 == p2:
                error = 0
            else:
                error = math.fabs(p2 - p1) / max(math.fabs(p1), math.fabs(p2))
            self.assertLess(error, 10 * self.error_max, "The two simulations have diverged")

    def test_get_state_set_state_oscillating_altitude(self):
        # taking actions only on the elevator command
        self.env.task.define_action([c.fcs_elevator_cmd_norm])
        self.env.reset()

        get_state = False
        actions = []
        state = None
        t = 0
        delta = 0

        kp = 0.005
        kd = 1

        # first flight
        while t <= self.tmax and self.env.sim.get_property_value(c.position_h_sl_ft) >= 200:
            if t >= 5 and get_state is False:
                # get_state after 5 seconds of simulation
                state = self.env.get_state()
                get_state = True

            # compute action
            last_delta = delta
            delta = self.env.sim.get_property_value(c.delta_altitude)
            action = [-(kp * delta + kd * (delta - last_delta))]

            if get_state:
                actions.append(action)

            self.env.step(action)
            t = self.env.get_sim_time()

        end_state_1 = self.env.get_state()

        # second flight
        self.env.set_state(state)
        for action in actions:
            self.env.step(action)
        end_state_2 = self.env.get_state()

        # compute error between two end states
        for prop in self.state_properties:
            p1 = end_state_1[prop]
            p2 = end_state_2[prop]
            if p1 == p2:
                error = 0
            else:
                error = math.fabs(p2 - p1) / max(math.fabs(p1), math.fabs(p2))
            self.assertLess(error, self.error_max, "The two simulations have diverged")
