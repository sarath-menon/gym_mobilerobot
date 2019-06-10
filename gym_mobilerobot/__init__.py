from gym.envs.registration import register

register(
    id='Mobilerobot-v0',
    entry_point='gym_mobilerobot.envs:MobileRobotGymEnv'
)
