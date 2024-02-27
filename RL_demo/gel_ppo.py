import base64
from torch.utils.tensorboard import SummaryWriter
import os
import numpy as np
import torch
import cv2
from scipy.spatial.transform import Rotation as R

from pyrfuniverse.envs.base_env import RFUniverseBaseEnv
from pyrfuniverse.side_channel.side_channel import (
    IncomingMessage,
    OutgoingMessage,
)
import warnings
warnings.filterwarnings("ignore", category=UserWarning)

import gym
from stable_baselines3.common.env_checker import check_env
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.utils import set_random_seed

from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.results_plotter import load_results, ts2xy, plot_results
from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback
from stable_baselines3.common import results_plotter


center_pos_x = 0
center_pos_y = 0
center_pos_z = 0

image_gel = None
success = False
collision = False
is_done = False

def GetCenterPos(msg: IncomingMessage):
    global center_pos_x, center_pos_y, center_pos_z
    center_pos_x = msg.read_float32()
    center_pos_y = msg.read_float32()
    center_pos_z = msg.read_float32()

def GetCollision(msg: IncomingMessage):
    global collision
    collision = msg.read_bool()

def GetSuccess(msg: IncomingMessage):
    global success
    success = msg.read_bool()

def GetImage(msg: IncomingMessage):
    global image_gel
    image_gel = base64.b64decode(msg.read_string())
    image_gel = np.frombuffer(image_gel, dtype=np.uint8)
    image_gel = cv2.imdecode(image_gel, cv2.IMREAD_COLOR)

def GetDone(msg: IncomingMessage):
    global is_done
    is_done = True
    
env_unity = RFUniverseBaseEnv(
    executable_file='@editor',
)

env_unity.asset_channel.AddListener('CenterPos', GetCenterPos)
env_unity.asset_channel.AddListener('Cross', GetSuccess)
env_unity.asset_channel.AddListener('Image', GetImage)
env_unity.asset_channel.AddListener('Done', GetDone)

for _ in range(500):
    env_unity.step()


env_unity.asset_channel.SendMessage('Reset', 0., 0.)
for _ in range(200):
    env_unity.step()
    

class TacIEnv(gym.Env):
    def __init__(self, env_unity):
        super(TacIEnv, self).__init__()
        self.env_unity = env_unity
        
        # Define action and observation space
        # They must be gym.spaces objects
        # Example when using discrete actions:
        self.action_space = gym.spaces.Box(low=-0.25, high=0.25, 
                                           shape=(2,), dtype=np.float32)
        # Example for using image as input (channel-first; channel-last also works):
        self.observation_space = gym.spaces.Box(low=-1280, high=1280,
                                                shape=(2,), dtype=np.int16)
        self.img_needle_ref = cv2.imread('needle_ref.png')
        self.img_needle_ref = cv2.resize(self.img_needle_ref, (1600, 1200))
        self.hole_ref = cv2.imread('hole_ref.png')
        self.hole_ref = cv2.resize(self.hole_ref, (1600, 1200))
        
        self.iteration = 0
        self.img_id = 0
        self.needle_pixel = np.array([600, 800])
        
    def step(self, action):
        
        depth_random = float(np.random.rand() * 0.1 - 0.05)
        self.env_unity.asset_channel.SendMessage('Step', float(action[0]), float(action[1]), depth_random)
        
        global image_gel, is_done
        is_done = False
        while not is_done:
            self.env_unity.step()
        
        self.env_unity.asset_channel.SendMessage('GetImage')
        self.env_unity.step()
        
        self.img_id += 1
        img_test_ref = image_gel
        
        gel_out = False
        if np.max(img_test_ref) < 1:
            gel_out = True
            print("Gel out!!! Reward: -100")
            return np.zeros((2,)), -100, True, {}
            

        img_test_ref[np.where(img_test_ref > 0.5 * np.max(img_test_ref))] = 255
        len_line = np.where(img_test_ref > 0)[0].shape[0]
        
        
        x_idx = int(np.mean(list(set(np.where(img_test_ref == 255)[0].tolist()))))
        y_idx = int(np.mean(list(set(np.where(img_test_ref == 255)[1].tolist()))))
        
        observation = np.array(self.needle_pixel - np.array([x_idx, y_idx]), dtype=np.int16)
        
        done = False
        
        hole_ref_inv = self.hole_ref / 2
        img_hole_line_inv = img_test_ref / 2
        
        img_line_ref = hole_ref_inv + img_hole_line_inv
        len_line_in_hole = np.where(img_line_ref > 200)[0].shape[0]
        
        if len_line_in_hole > 0.5 * len_line:
            done = True
            cv2.imwrite('./success_img/{}_success.png'.format(self.iteration), image_gel)
        
        reward = self.get_reward(observation, done, collision or gel_out)
        if done: print("Success!!! Reward = 1000")
        else: print("Reward =", reward)
        
        info = dict()
        
        self.iteration += 1
        
        return observation, reward, done or collision or self.iteration >= 1000, info

    def get_reward(self, observation, done, collision_or_gel_out):
        if done:
            return 100
        if collision_or_gel_out:
            return -100
        else:
            return -np.linalg.norm([observation[0] * 4 / 3, observation[1]]) / np.linalg.norm([self.needle_pixel[0] * 4 / 3, self.needle_pixel[1]])
        
    def reset(self):
        
        self.iteration = 0
        
        while True:
            x_random = float(np.random.rand() * 0.9 + 0.6)
            y_random = float(np.random.rand() * 1.6 - 0.8)
            
            self.env_unity.asset_channel.SendMessage('Reset', x_random, y_random)
            
            global is_done, image_gel
            is_done = False
            while not is_done:
                self.env_unity.step()
            
            self.env_unity.asset_channel.SendMessage('GetImage')
            self.env_unity.step()
            
            img_test_ref = image_gel
            if np.where(img_test_ref > 0.5 * np.max(img_test_ref))[0].shape[0] != 0:
                break

        img_test_ref[np.where(img_test_ref > 0.5 * np.max(img_test_ref))] = 255
        
        x_idx = int(np.mean(list(set(np.where(img_test_ref == 255)[0].tolist()))))
        y_idx = int(np.mean(list(set(np.where(img_test_ref == 255)[1].tolist()))))
        
        observation = np.array(self.needle_pixel - np.array([x_idx, y_idx]), dtype=np.int16)
        print("============Reset============")

        return observation


def make_env(rank, seed=0):
    """
    Utility function for multiprocessed env.

    :param env_id: (str) the environment ID
    :param num_env: (int) the number of environments you wish to have in subprocesses
    :param seed: (int) the inital seed for RNG
    :param rank: (int) index of the subprocess
    """
    def _init():
        env = TacIEnv(env_unity=env_unity)
        env.seed(seed + rank)

        return env
    set_random_seed(seed)
    return _init

log_dir = 'logs_zip'
tensorboard_log = 'logs_tf'
num_cpu = 1
env = DummyVecEnv([make_env(i) for i in range(num_cpu)])

model_ppo = PPO("MlpPolicy", env, verbose=1, n_steps=1000,
            tensorboard_log=tensorboard_log)

checkpoint_callback = CheckpointCallback(
  save_freq=2000,
  save_path=log_dir,
  name_prefix="ppo_gel",
  save_replay_buffer=True,
  save_vecnormalize=True,
)

total_timesteps = 10000
for i in range(5):
    model_ppo.learn(total_timesteps=total_timesteps, callback=checkpoint_callback, tb_log_name="{}".format((i+1) * total_timesteps), reset_num_timesteps=False)
model_ppo.learn(total_timesteps=total_timesteps, callback=checkpoint_callback, reset_num_timesteps=False)

