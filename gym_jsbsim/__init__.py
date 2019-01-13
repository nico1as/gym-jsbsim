from gym.envs.registration import register
#
# Env registration
#
register(
    id='MinecraftDefaultWorld1-v0',
    entry_point='gym_minecraft.envs:MinecraftEnv',
    kwargs={'mission_file': 'default_world_1.xml'},
    #tags={'wrapper_config.TimeLimit.max_episode_steps': 6060},
    #timestep_limit=6060,
    reward_threshold=1000
)

register(
    id='MinecraftDefaultWorld1-v0',
    entry_point='gym_minecraft.envs:MinecraftEnv',
    kwargs={'mission_file': 'default_world_1.xml'},
    #tags={'wrapper_config.TimeLimit.max_episode_steps': 6060},
    #timestep_limit=6060,
    reward_threshold=1000
)
