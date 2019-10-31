import gym
import gym_jsbsim


def random_agent():

	env = gym.make("GymJsbsim-HeadingControlTask-v0")
	env.reset()
	done = False

	while not done:
		
		action = env.action_space.sample()
		state, reward, done, _ = env.step(action)

		print("action =", action, " ---> State =", state, " : Reward =", reward)
		
if __name__ == "__main__":
	random_agent()
