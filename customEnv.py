import gym
import rware


from layout import layout_bigstreet, layout_smallstreet


if __name__=="__main__":
    env = gym.make("rware-tiny-4ag-v1", layout=layout_smallstreet)
    obs = env.reset()
    while True:
        env.render()
        actions = env.action_space.sample()
        env.step(actions)