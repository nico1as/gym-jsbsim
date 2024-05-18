import unittest
import random
import gym_jsbsim
from gym_jsbsim import Catalog as c
from gym_jsbsim.catalogs.utils import reduce_reflex_angle_deg


class TestPropertyUpdates(unittest.TestCase):
    def setUp(self):
        self.env = gym_jsbsim.make("GymJsbsim-HeadingControlTask-v0")
        self.env.reset()

    def tearDown(self):
        self.env.close()

    def test_update_throttles(self):
        new_throttle_cmd = random.random() * 0.9
        self.env.sim.set_property_value(c.fcs_throttle_cmd_norm, new_throttle_cmd)
        cur_throttle_cmd = self.env.sim.get_property_value(c.fcs_throttle_cmd_norm)
        cur_throttle_cmd_1 = self.env.sim.get_property_value(c.fcs_throttle_cmd_norm_1)
        self.assertEqual(cur_throttle_cmd, cur_throttle_cmd_1, "Unequal throttle commands")

    def test_update_throttle_cmd_dir(self):
        old_throttle_cmd = self.env.sim.get_property_value(c.fcs_throttle_cmd_norm)
        self.env.sim.set_property_value(c.throttle_cmd_dir, 2)
        cur_throttle_cmd = self.env.sim.get_property_value(c.fcs_throttle_cmd_norm)
        incr_throttle = self.env.sim.get_property_value(c.incr_throttle)
        self.assertEqual(cur_throttle_cmd, old_throttle_cmd + incr_throttle, "Throttle was not updated correctly")

    def test_update_delta_altitude(self):
        alt = self.env.sim.get_property_value(c.position_h_sl_ft)
        target_alt = self.env.sim.get_property_value(c.target_altitude_ft)
        self.assertEqual(
            target_alt - alt, self.env.sim.get_property_value(c.delta_altitude), "Delta altitude incorrect"
        )
        new_alt = 11000
        self.env.sim.set_property_value(c.position_h_sl_ft, new_alt)
        self.assertEqual(
            target_alt - new_alt, self.env.sim.get_property_value(c.delta_altitude), "Delta altitude incorrect"
        )

    def test_update_delta_heading(self):
        heading = self.env.sim.get_property_value(c.attitude_psi_deg)
        target_heading = self.env.sim.get_property_value(c.target_heading_deg)
        delta_heading = reduce_reflex_angle_deg(target_heading - heading)
        self.assertEqual(delta_heading, self.env.sim.get_property_value(c.delta_heading), "Delta heading incorrect")
        self.env.step([1, 1, 1, 1])
        heading = self.env.sim.get_property_value(c.attitude_psi_deg)
        delta_heading = reduce_reflex_angle_deg(target_heading - heading)
        self.assertEqual(delta_heading, self.env.sim.get_property_value(c.delta_heading), "Delta heading incorrect")

    def test_update_detect_extreme_state(self):
        self.assertFalse(
            bool(self.env.sim.get_property_value(c.detect_extreme_state)), "Should not detect extreme state"
        )
        self.env.sim.jsbsim_exec.set_property_value(c.position_h_sl_ft.name_jsbsim, 1e10)
        self.assertTrue(bool(self.env.sim.get_property_value(c.detect_extreme_state)), "Should detect extreme state")

    def test_update_center_brake(self):
        new_brake_cmd = random.random()
        self.env.sim.set_property_value(c.fcs_center_brake_cmd_norm, new_brake_cmd)
        self.assertEqual(
            new_brake_cmd, self.env.sim.get_property_value(c.fcs_left_brake_cmd_norm), "Left brake was not updated"
        )
        self.assertEqual(
            new_brake_cmd, self.env.sim.get_property_value(c.fcs_right_brake_cmd_norm), "Right brake was not updated"
        )
