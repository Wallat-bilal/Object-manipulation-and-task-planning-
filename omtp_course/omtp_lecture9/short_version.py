import gym
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3 import DQN
# CartPole-v1
env = gym.make('CartPole-v1')
model = DQN("MlpPolicy", env, verbose=1, tensorboard_log='./logs')
# Set whether to train or test
train_model = False
model_filename = 'dqn_cartpole_100000'
#model = DQN.load(model_filename, env, verbose=1, tensorboard_log='./logs')
# Train model
if train_model:
    model.learn(total_timesteps=1000000, tb_log_name="PPO")
    model.save(model_filename)
# Test model
else:
    # del model # remove to demonstrate saving and loading
    #model = DQN.load(model_filename)
    # Enjoy trained agent
    obs = env.reset()
    while True:
        action, _states = model.predict(obs)
        obs, rewards, dones, info = env.step(action)
        env.render()