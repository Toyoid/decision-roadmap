import numpy as np
import torch
import torch.nn as nn
from torch.distributions.normal import Normal


def layer_init(layer, std=np.sqrt(2), bias_const=0.0):
    torch.nn.init.orthogonal_(layer.weight, std)
    torch.nn.init.constant_(layer.bias, bias_const)
    return layer


class MLPPPONets(nn.Module):
    """
    MLP Actor-critic neural networks of PPO with continuous action space
    """
    def __init__(self, observation_dim, action_dim):
        super().__init__()
        self.critic = nn.Sequential(
            layer_init(nn.Linear(observation_dim, 64)),
            nn.Tanh(),
            layer_init(nn.Linear(64, 64)),
            nn.Tanh(),
            layer_init(nn.Linear(64, 1), std=1.0),
        )
        self.actor_mean = nn.Sequential(
            layer_init(nn.Linear(observation_dim, 64)),
            nn.Tanh(),
            layer_init(nn.Linear(64, 64)),
            nn.Tanh(),
            layer_init(nn.Linear(64, action_dim), std=0.01),
        )
        self.actor_logstd = nn.Parameter(torch.zeros(1, action_dim))

    def get_value(self, x):
        return self.critic(x)

    def get_action_and_value(self, x, action=None):
        action_mean = self.actor_mean(x)
        action_logstd = self.actor_logstd.expand_as(action_mean)
        action_std = torch.exp(action_logstd)
        probs = Normal(action_mean, action_std)
        if action is None:
            action = probs.sample()
        return action, probs.log_prob(action).sum(1), probs.entropy().sum(1), self.critic(x)


class CNNPPONets(nn.Module):
    """
    Actor-critic networks of PPO with CNN for processing depth image
    """
    def __init__(self, img_obs_shape, robot_obs_dim, action_dim, action_scale):
        super().__init__()
        self.action_scale = action_scale
        self.cnn = nn.Sequential(
            layer_init(nn.Conv2d(in_channels=img_obs_shape[0], out_channels=32, kernel_size=(6, 8), stride=8)),
            nn.ReLU(),
            layer_init(nn.Conv2d(in_channels=32, out_channels=64, kernel_size=(3, 4), stride=3)),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=3, stride=2),
            # layer_init(nn.Conv2d(in_channels=64, out_channels=64, kernel_size=3, stride=2)),
            # nn.ReLU(),
            nn.Flatten(),
            layer_init(nn.Linear(64 * 9 * 12, 512)),
            nn.ReLU()
        )
        '''set the action space to (right wheel, left wheel, y-axis)? and maybe raise the r-goal and r-collision'''
        # X = torch.randn(1, 10, 480, 640)
        # for layer in self.cnn:
        #     X = layer(X)
        #     print(layer.__class__.__name__, 'output shape:\t', X.shape)
        # raise

        # good architecture 1: CNN + CNN + Maxpool + flatten + linear, actor & critic (256)

        self.fcn = nn.Sequential(
            layer_init(nn.Linear(robot_obs_dim, 128)),
            nn.ReLU(),
        )
        self.critic = nn.Sequential(     # whether to share AC networks?
            layer_init(nn.Linear(512+128, 256)),  # need wider layers (256)
            # nn.Tanh(),
            nn.ReLU(),
            layer_init(nn.Linear(256, 256)),
            nn.ReLU(),
            # nn.Tanh(),
            layer_init(nn.Linear(256, 1), std=1.0),
        )
        self.actor_mean = nn.Sequential(
            layer_init(nn.Linear(512+128, 256)),
            # nn.Tanh(),
            nn.ReLU(),
            layer_init(nn.Linear(256, 256)),
            nn.ReLU(),
            # nn.Tanh(),
            layer_init(nn.Linear(256, action_dim), std=0.01),
            nn.Tanh(),
        )
        self.actor_logstd = nn.Parameter(torch.zeros(1, action_dim))

    def get_value(self, img, obs_robot):
        img_embedding = self.cnn(img)
        robot_embedding = self.fcn(obs_robot)
        obs_merge = torch.cat((robot_embedding, img_embedding), dim=1)
        return self.critic(obs_merge)

    def get_action_and_value(self, img, obs_robot, action=None):
        img_embedding = self.cnn(img)
        robot_embedding = self.fcn(obs_robot)
        obs_merge = torch.cat((robot_embedding, img_embedding), dim=1)
        action_mean = self.actor_mean(obs_merge)
        action_mean = action_mean * self.action_scale  # remap mean of the action from Tanh (-1,1)
        action_logstd = self.actor_logstd.expand_as(action_mean)
        action_std = torch.exp(action_logstd)
        probs = Normal(action_mean, action_std)
        if action is None:
            action = probs.sample()
        # action_mean, action_std is for debugging, not necessary
        return action, probs.log_prob(action).sum(1), probs.entropy().sum(1), self.critic(obs_merge), action_mean, action_std
