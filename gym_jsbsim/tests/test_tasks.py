import unittest
import gym_jsbsim


class TestValidTasks(unittest.TestCase):
    def test_valid_action(self):
        for name, task in gym_jsbsim.TASKS.items():
            self.assertIsNotNone(task.action_var, f"Actions not defined in {name}")
            for prop in task.action_var:
                self.assertNotEqual(prop.min, float("-inf"), "Action property unbounded")
                self.assertNotEqual(prop.max, float("+inf"), "Action property unbounded")

    def test_valid_observation(self):
        for name, task in gym_jsbsim.TASKS.items():
            self.assertIsNotNone(task.state_var, f"Observations not defined in {name}")
            for prop in task.state_var:
                self.assertNotEqual(prop.min, float("-inf"), "State property not unbounded")
                self.assertNotEqual(prop.max, float("+inf"), "State property not unbounded")

    def test_valid_jsbsim_frequency(self):
        for name, task in gym_jsbsim.TASKS.items():
            self.assertGreaterEqual(task.jsbsim_freq, 60, f"Jsbsim frequency too low in {name}")

    def test_valid_init_conditions(self):
        for name, task in gym_jsbsim.TASKS.items():
            if task.init_conditions:
                for prop, value in task.init_conditions.items():
                    self.assertGreaterEqual(value, prop.min, f"Initial value of {prop} out of bounds in {name}")
                    self.assertLessEqual(value, prop.max, f"Initial value of {prop} out of bounds in {name}")
