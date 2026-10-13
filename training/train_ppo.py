"""
train_ppo.py

Self-contained PyTorch PPO (Proximal Policy Optimization) algorithm
for training the TwoWheelRobot on Section 1 Obstacle Navigation.
Exports trained weights to JSON for zero-dependency inference in main_controller.
"""

import os
import json
import math
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.distributions import Normal

from webots_env import WebotsSection1Env

class ActorCritic(nn.Module):
    def __init__(self, obs_dim=11, act_dim=2):
        super().__init__()
        # Actor Network (determines mean action)
        self.actor = nn.Sequential(
            nn.Linear(obs_dim, 64),
            nn.Tanh(),
            nn.Linear(64, 64),
            nn.Tanh(),
            nn.Linear(64, act_dim),
            nn.Tanh()
        )
        # Log standard deviation (learnable parameter)
        self.log_std = nn.Parameter(torch.ones(act_dim) * -0.5)
        
        # Critic Network (estimates state value V(s))
        self.critic = nn.Sequential(
            nn.Linear(obs_dim, 64),
            nn.Tanh(),
            nn.Linear(64, 64),
            nn.Tanh(),
            nn.Linear(64, 1)
        )

    def forward(self, obs):
        mean = self.actor(obs)
        std = torch.exp(self.log_std)
        value = self.critic(obs)
        return mean, std, value

    def get_action(self, obs):
        with torch.no_grad():
            mean, std, value = self.forward(obs)
            dist = Normal(mean, std)
            action = dist.sample()
            log_prob = dist.log_prob(action).sum(dim=-1)
        return action.cpu().numpy(), log_prob.cpu().numpy(), value.item()

    def evaluate(self, obs, actions):
        mean, std, values = self.forward(obs)
        dist = Normal(mean, std)
        log_probs = dist.log_prob(actions).sum(dim=-1)
        entropy = dist.entropy().sum(dim=-1)
        return log_probs, values.squeeze(-1), entropy

    def export_weights(self, output_path):
        """Exports Actor weights to JSON for NumPy-only evaluation."""
        layers = [m for m in self.actor if isinstance(m, nn.Linear)]
        export_dict = {
            "w1": layers[0].weight.detach().cpu().numpy().T.tolist(),
            "b1": layers[0].bias.detach().cpu().numpy().tolist(),
            "w2": layers[1].weight.detach().cpu().numpy().T.tolist(),
            "b2": layers[1].bias.detach().cpu().numpy().tolist(),
            "w_out": layers[2].weight.detach().cpu().numpy().T.tolist(),
            "b_out": layers[2].bias.detach().cpu().numpy().tolist(),
        }
        with open(output_path, "w", encoding="utf-8") as f:
            json.dump(export_dict, f, indent=2)
        print(f"[EXPORT] Saved policy weights to {output_path}")

def compute_gae(rewards, values, dones, gamma=0.99, lam=0.95):
    advantages = []
    gae = 0.0
    values = values + [0.0]
    for step in reversed(range(len(rewards))):
        delta = rewards[step] + gamma * values[step + 1] * (1.0 - dones[step]) - values[step]
        gae = delta + gamma * lam * (1.0 - dones[step]) * gae
        advantages.insert(0, gae)
    returns = [adv + val for adv, val in zip(advantages, values[:-1])]
    return advantages, returns

def train():
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Training PPO on device: {device}")
    
    env = WebotsSection1Env()
    policy = ActorCritic(obs_dim=11, act_dim=2).to(device)
    optimizer = optim.Adam(policy.parameters(), lr=3e-4)
    
    # Training Hyperparameters
    total_timesteps = 100_000
    rollout_steps = 1024
    batch_size = 64
    epochs = 10
    clip_eps = 0.2
    
    export_target = os.path.join(
        os.path.dirname(__file__), "..", "controllers", "main_controller", "policy_weights.json"
    )
    
    global_step = 0
    obs, _ = env.reset()
    
    while global_step < total_timesteps:
        # 1. Collect rollout
        rollout_obs = []
        rollout_actions = []
        rollout_log_probs = []
        rollout_rewards = []
        rollout_dones = []
        rollout_values = []
        
        ep_rewards = []
        curr_ep_reward = 0.0
        
        for _ in range(rollout_steps):
            obs_tensor = torch.tensor(obs, dtype=torch.float32, device=device)
            action, log_prob, value = policy.get_action(obs_tensor)
            
            next_obs, reward, done, info = env.step(action)
            
            rollout_obs.append(obs)
            rollout_actions.append(action)
            rollout_log_probs.append(log_prob)
            rollout_rewards.append(reward)
            rollout_dones.append(float(done))
            rollout_values.append(value)
            
            curr_ep_reward += reward
            global_step += 1
            
            if done:
                ep_rewards.append(curr_ep_reward)
                curr_ep_reward = 0.0
                obs, _ = env.reset()
            else:
                obs = next_obs
                
        # 2. Compute Generalized Advantage Estimation (GAE)
        advantages, returns = compute_gae(rollout_rewards, rollout_values, rollout_dones)
        
        b_obs = torch.tensor(np.array(rollout_obs), dtype=torch.float32, device=device)
        b_actions = torch.tensor(np.array(rollout_actions), dtype=torch.float32, device=device)
        b_log_probs = torch.tensor(np.array(rollout_log_probs), dtype=torch.float32, device=device)
        b_adv = torch.tensor(np.array(advantages), dtype=torch.float32, device=device)
        b_returns = torch.tensor(np.array(returns), dtype=torch.float32, device=device)
        
        # Normalize advantages
        b_adv = (b_adv - b_adv.mean()) / (b_adv.std() + 1e-8)
        
        # 3. Optimize policy and value network
        dataset_size = len(rollout_obs)
        for _ in range(epochs):
            indices = np.random.permutation(dataset_size)
            for start in range(0, dataset_size, batch_size):
                end = start + batch_size
                batch_idx = indices[start:end]
                
                log_probs, values, entropy = policy.evaluate(b_obs[batch_idx], b_actions[batch_idx])
                
                ratios = torch.exp(log_probs - b_log_probs[batch_idx])
                surr1 = ratios * b_adv[batch_idx]
                surr2 = torch.clamp(ratios, 1.0 - clip_eps, 1.0 + clip_eps) * b_adv[batch_idx]
                actor_loss = -torch.min(surr1, surr2).mean()
                
                critic_loss = 0.5 * ((values - b_returns[batch_idx]) ** 2).mean()
                loss = actor_loss + 0.5 * critic_loss - 0.01 * entropy.mean()
                
                optimizer.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(policy.parameters(), max_norm=0.5)
                optimizer.step()
                
        mean_reward = np.mean(ep_rewards) if ep_rewards else curr_ep_reward
        print(f"[Step {global_step}/{total_timesteps}] Mean Episode Reward: {mean_reward:.2f}")
        
        # Checkpoint and export weights
        torch.save(policy.state_dict(), os.path.join(os.path.dirname(__file__), "ppo_checkpoint.pt"))
        policy.export_weights(export_target)

if __name__ == "__main__":
    train()
