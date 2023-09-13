from rdlvy.robot_delivery import *
from layout import layout_2way, layout_2way_simple
from tqdm import tqdm

#env = RobotDelivery(n_agents=4,
#                msg_bits=0,
#                sensor_range=3,
#                request_queue_size=5,
#                max_inactivity_steps=10000,
#                max_steps=10000,
#                reward_type=RewardType.TWO_STAGE,
#                layout=layout_2way_simple,
#                block_size="big",
#                observation_type=ObservationType.IMAGE,
#                use_full_obs=True
#                )
env = RobotDelivery(n_agents=30,
                msg_bits=0,
                sensor_range=10,
                request_queue_size=20,
                max_inactivity_steps=10000,
                max_steps=10000,
                reward_type=RewardType.TWO_STAGE,
                layout=layout_2way,
                block_size="small",
                observation_type=ObservationType.IMAGE,
                use_full_obs=True
                )

state = env.reset()

for _ in tqdm(range(1000000)):
    env.render()
    actions = env.action_space.sample()
    next_obs, reward, dones, info = env.step(actions)