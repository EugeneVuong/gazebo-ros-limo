import torch
import torch.nn as nn
import torch.optim as optim
from torch.distributions import Normal

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

class Actor(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_dim=64):
        super(Actor, self).__init__()
        self.fc1 = nn.Linear(state_dim, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.mu = nn.Linear(hidden_dim, action_dim)
        self.log_std = nn.Parameter(torch.zeros(action_dim))

    def forward(self, state):
        x = torch.tanh(self.fc1(state))
        x = torch.tanh(self.fc2(x))
        mu = self.mu(x)
        std = torch.exp(self.log_std)
        std = torch.clamp(std, 1e-3, 1.0)
        return mu, std

class Critic(nn.Module):
    def __init__(self, state_dim, hidden_dim=64):
        super(Critic, self).__init__()
        self.fc1 = nn.Linear(state_dim, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.value = nn.Linear(hidden_dim, 1)

    def forward(self, state):
        x = torch.tanh(self.fc1(state))
        x = torch.tanh(self.fc2(x))
        value = self.value(x)
        return value

class PPOAgent:
    def __init__(self, state_dim, action_dim, lr=3e-4, gamma=0.99, eps_clip=0.2, K_epochs=4):
        self.actor = Actor(state_dim, action_dim)
        self.critic = Critic(state_dim)
        self.optimizer = optim.Adam(list(self.actor.parameters()) + list(self.critic.parameters()), lr=lr)
        self.gamma = gamma
        self.eps_clip = eps_clip
        self.K_epochs = K_epochs

        self.states = []
        self.actions = []
        self.logprobs = []
        self.rewards = []
        self.dones = []

        # move to device
        self.device = device
        self.actor.to(self.device)
        self.critic.to(self.device)

    def select_action(self, state):
        state = torch.FloatTensor(state).to(self.device)
        # sanitize state to remove nan or inf
        state = torch.nan_to_num(state, nan=0.0, posinf=0.0, neginf=0.0)
        mu, std = self.actor(state)
        dist = Normal(mu, std)
        action = dist.sample()
        logprob = dist.log_prob(action).sum()
        self.states.append(state)
        self.actions.append(action)
        self.logprobs.append(logprob)
        return action.detach().cpu().numpy()

    def store_reward(self, reward, done):
        self.rewards.append(reward)
        self.dones.append(done)

    def update(self):
        # calculate discounted returns
        returns = []
        discounted = 0
        for reward, done in zip(reversed(self.rewards), reversed(self.dones)):
            if done:
                discounted = 0
            discounted = reward + self.gamma * discounted
            returns.insert(0, discounted)
        returns = torch.FloatTensor(returns).to(self.device)
        states = torch.stack(self.states).to(self.device)
        actions = torch.stack(self.actions).to(self.device)
        old_logprobs = torch.stack(self.logprobs).detach().to(self.device)

        # advantage estimation
        values = self.critic(states).squeeze()
        advantages = returns - values.detach()

        # optimize policy for K epochs
        for _ in range(self.K_epochs):
            mu, std = self.actor(states)
            dist = Normal(mu, std)
            logprobs = dist.log_prob(actions).sum(dim=1)
            entropy = dist.entropy().sum(dim=1)
            ratios = torch.exp(logprobs - old_logprobs)
            surr1 = ratios * advantages
            surr2 = torch.clamp(ratios, 1 - self.eps_clip, 1 + self.eps_clip) * advantages
            loss = -torch.min(surr1, surr2).mean() + 0.5 * (returns - self.critic(states).squeeze()).pow(2).mean() - 0.01 * entropy.mean()

            self.optimizer.zero_grad()
            loss.backward()
            self.optimizer.step()

        # clear memory
        self.states = []
        self.actions = []
        self.logprobs = []
        self.rewards = []
        self.dones = []
