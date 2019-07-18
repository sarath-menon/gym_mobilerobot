from gym.envs.registration import register

register(
    id='Mobilerobot_sim_odom-v0',
    entry_point='gym_mobilerobot.envs:MobileRobotGymEnv_sim_odom'
)

register(
    id='Mobilerobot_sim_amcl-v0',
    entry_point='gym_mobilerobot.envs:MobileRobotGymEnv_sim_amcl'
)

register(
    id='Mobilerobot_real_amcl-v0',
    entry_point='gym_mobilerobot.envs:MobileRobotGymEnv_real_amcl'
)

register(
    id='Mobilerobot_real_fusion-v0',
    entry_point='gym_mobilerobot.envs:MobileRobotGymEnv_real_fusion'
)
