import random
import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import numpy as np
from collections import deque
from tqdm import tqdm
import random
import math
import torch.nn.init as init
from torch.distributions import Categorical
class A2CAgent(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(A2CAgent, self).__init__()

        self.actor = nn.Sequential(
            nn.Linear(state_dim, 64),
            nn.ReLU(),
            nn.Linear(64, action_dim),
            nn.Softmax(dim=-1)
        )
        self.critic = nn.Sequential(
            nn.Linear(state_dim, 64),
            nn.ReLU(),
            nn.Linear(64, 1)
        )

        self.optimizer = optim.Adam(self.parameters(), lr=1e-3)
        self.saved_log_probs = []
        self.rewards = []
        self.values = []
        self.gamma = 0.99

    def forward(self, state):
        action_prob = self.actor(state)
        state_value = self.critic(state)

        return action_prob, state_value

    def choose_action(self, state):
        state = torch.tensor(state, dtype=torch.float32).reshape(1,-1)
        action_probs, state_value = self(state)
        action_dist = Categorical(action_probs)
        action = action_dist.sample()
        self.saved_log_probs.append(action_dist.log_prob(action))
        self.values.append(state_value)

        return action.item()

    def remember(self, reward):
        self.rewards.append(reward)

    def update(self, state, done):
        state = torch.tensor(state, dtype=torch.float32).reshape(1,-1)
        _, value = self(state)

        self.log_probs = torch.cat(self.saved_log_probs)
        self.values = torch.cat(self.values)
        self.rewards = torch.tensor(self.rewards, dtype=torch.float32).to(self.values.device)

        returns = self.rewards + self.gamma * self.values[1:] * (1 - torch.tensor(done, dtype=torch.float32).to(self.values.device))
        returns = torch.cat((returns, value))
        advantage = returns[:-1] - self.values[:-1]

        actor_loss = -(self.log_probs * advantage.detach()).mean()
        critic_loss = advantage.pow(2).mean()

        self.optimizer.zero_grad()
        (actor_loss + critic_loss).backward()
        self.optimizer.step()

        del self.saved_log_probs[:]
        self.values = []
        self.rewards = []
