# Reinforcement Learning for Control

## Introduction to Reinforcement Learning in Robotics

Reinforcement Learning (RL) has emerged as a powerful paradigm for learning complex robotic behaviors directly from interaction with the environment. Unlike traditional control methods that rely on explicit mathematical models, RL enables robots to learn optimal control policies through trial and error, making it particularly suitable for complex, high-dimensional tasks where analytical solutions are difficult to obtain.

## RL Fundamentals for Robotics

### The RL Framework

In robotics, the RL framework consists of:

- **Agent**: The robot or control system
- **Environment**: The physical world or simulation
- **State (s)**: Robot configuration, sensor readings, task context
- **Action (a)**: Motor commands, joint torques, velocities
- **Reward (r)**: Feedback signal indicating task success
- **Policy (π)**: Mapping from states to actions

```
Environment → State (s) → Agent → Action (a) → Environment → Reward (r)
     ↑                                           ↓
     └─────────────────────────────────────────────┘
```

### Markov Decision Process (MDP) Formulation

The robotic control problem can be formulated as an MDP:
- **State Space (S)**: Continuous or discrete states representing robot configuration and environment
- **Action Space (A)**: Continuous (torques, velocities) or discrete (motion primitives) actions
- **Transition Dynamics (P)**: Probability of transitioning to next state given current state and action
- **Reward Function (R)**: Scalar feedback signal
- **Discount Factor (γ)**: Trade-off between immediate and future rewards

## Deep Reinforcement Learning Algorithms

### Deep Q-Network (DQN) for Discrete Actions

```python
import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
import random
from collections import deque

class DQN(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_dim=256):
        super(DQN, self).__init__()

        self.network = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, action_dim)
        )

    def forward(self, state):
        return self.network(state)

class DQNAgent:
    def __init__(self, state_dim, action_dim, lr=1e-4, gamma=0.99, epsilon=1.0, epsilon_decay=0.995, epsilon_min=0.01):
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.gamma = gamma
        self.epsilon = epsilon
        self.epsilon_decay = epsilon_decay
        self.epsilon_min = epsilon_min

        # Neural networks
        self.q_network = DQN(state_dim, action_dim)
        self.target_network = DQN(state_dim, action_dim)
        self.optimizer = optim.Adam(self.q_network.parameters(), lr=lr)

        # Replay buffer
        self.memory = deque(maxlen=100000)
        self.batch_size = 32

        # Update target network
        self.update_target_network()

    def update_target_network(self):
        """Copy weights from main network to target network"""
        self.target_network.load_state_dict(self.q_network.state_dict())

    def remember(self, state, action, reward, next_state, done):
        """Store experience in replay buffer"""
        self.memory.append((state, action, reward, next_state, done))

    def act(self, state):
        """Choose action using epsilon-greedy policy"""
        if np.random.random() <= self.epsilon:
            return random.randrange(self.action_dim)

        state_tensor = torch.FloatTensor(state).unsqueeze(0)
        q_values = self.q_network(state_tensor)
        return np.argmax(q_values.cpu().data.numpy())

    def replay(self):
        """Train the network on a batch of experiences"""
        if len(self.memory) < self.batch_size:
            return

        batch = random.sample(self.memory, self.batch_size)
        states = torch.FloatTensor([e[0] for e in batch])
        actions = torch.LongTensor([e[1] for e in batch])
        rewards = torch.FloatTensor([e[2] for e in batch])
        next_states = torch.FloatTensor([e[3] for e in batch])
        dones = torch.BoolTensor([e[4] for e in batch])

        current_q_values = self.q_network(states).gather(1, actions.unsqueeze(1))
        next_q_values = self.target_network(next_states).max(1)[0].detach()
        target_q_values = rewards + (self.gamma * next_q_values * ~dones)

        loss = nn.MSELoss()(current_q_values.squeeze(), target_q_values)

        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay
```

### Deep Deterministic Policy Gradient (DDPG) for Continuous Actions

```python
import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
from collections import deque
import random

class Actor(nn.Module):
    def __init__(self, state_dim, action_dim, max_action, hidden_dim=256):
        super(Actor, self).__init__()

        self.network = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, action_dim),
            nn.Tanh()
        )

        self.max_action = max_action

    def forward(self, state):
        return self.max_action * self.network(state)

class Critic(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_dim=256):
        super(Critic, self).__init__()

        self.network = nn.Sequential(
            nn.Linear(state_dim + action_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, 1)
        )

    def forward(self, state, action):
        return self.network(torch.cat([state, action], dim=1))

class DDPGAgent:
    def __init__(self, state_dim, action_dim, max_action, lr_actor=1e-4, lr_critic=1e-3,
                 gamma=0.99, tau=0.005, noise_std=0.2):
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.max_action = max_action
        self.gamma = gamma
        self.tau = tau
        self.noise_std = noise_std

        # Networks
        self.actor = Actor(state_dim, action_dim, max_action)
        self.actor_target = Actor(state_dim, action_dim, max_action)
        self.critic = Critic(state_dim, action_dim)
        self.critic_target = Critic(state_dim, action_dim)

        # Optimizers
        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=lr_actor)
        self.critic_optimizer = optim.Adam(self.critic.parameters(), lr=lr_critic)

        # Replay buffer
        self.memory = deque(maxlen=100000)
        self.batch_size = 100

        # Initialize target networks
        self.actor_target.load_state_dict(self.actor.state_dict())
        self.critic_target.load_state_dict(self.critic.state_dict())

    def select_action(self, state, add_noise=True):
        """Select action with optional noise for exploration"""
        state_tensor = torch.FloatTensor(state).unsqueeze(0)
        action = self.actor(state_tensor).cpu().data.numpy().flatten()

        if add_noise:
            noise = np.random.normal(0, self.noise_std, size=self.action_dim)
            action = action + noise
            action = np.clip(action, -self.max_action, self.max_action)

        return action

    def remember(self, state, action, reward, next_state, done):
        """Store experience in replay buffer"""
        self.memory.append((state, action, reward, next_state, done))

    def update(self):
        """Update actor and critic networks"""
        if len(self.memory) < self.batch_size:
            return

        batch = random.sample(self.memory, self.batch_size)
        states = torch.FloatTensor([e[0] for e in batch])
        actions = torch.FloatTensor([e[1] for e in batch])
        rewards = torch.FloatTensor([e[2] for e in batch]).unsqueeze(1)
        next_states = torch.FloatTensor([e[3] for e in batch])
        dones = torch.BoolTensor([e[4] for e in batch]).unsqueeze(1)

        # Critic update
        next_actions = self.actor_target(next_states)
        next_q_values = self.critic_target(next_states, next_actions)
        target_q_values = rewards + (self.gamma * next_q_values * ~dones)

        current_q_values = self.critic(states, actions)
        critic_loss = nn.MSELoss()(current_q_values, target_q_values.detach())

        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        self.critic_optimizer.step()

        # Actor update
        predicted_actions = self.actor(states)
        actor_loss = -self.critic(states, predicted_actions).mean()

        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        self.actor_optimizer.step()

        # Soft update target networks
        self.soft_update(self.actor_target, self.actor, self.tau)
        self.soft_update(self.critic_target, self.critic, self.tau)

    def soft_update(self, target_network, source_network, tau):
        """Soft update of target network parameters"""
        for target_param, param in zip(target_network.parameters(), source_network.parameters()):
            target_param.data.copy_(tau * param.data + (1 - tau) * target_param.data)
```

### Twin Delayed DDPG (TD3) - Advanced Continuous Control

```python
class TD3Critic(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_dim=256):
        super(TD3Critic, self).__init__()

        # Two Q-networks for double Q-learning
        self.q1 = nn.Sequential(
            nn.Linear(state_dim + action_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, 1)
        )

        self.q2 = nn.Sequential(
            nn.Linear(state_dim + action_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, 1)
        )

    def forward(self, state, action):
        sa = torch.cat([state, action], dim=1)
        q1 = self.q1(sa)
        q2 = self.q2(sa)
        return q1, q2

    def Q1(self, state, action):
        sa = torch.cat([state, action], dim=1)
        return self.q1(sa)

class TD3Agent:
    def __init__(self, state_dim, action_dim, max_action, lr_actor=1e-4, lr_critic=1e-3,
                 gamma=0.99, tau=0.005, policy_noise=0.2, noise_clip=0.5, policy_freq=2):
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.max_action = max_action
        self.gamma = gamma
        self.tau = tau
        self.policy_noise = policy_noise
        self.noise_clip = noise_clip
        self.policy_freq = policy_freq
        self.total_it = 0

        # Networks
        self.actor = Actor(state_dim, action_dim, max_action)
        self.actor_target = Actor(state_dim, action_dim, max_action)
        self.critic = TD3Critic(state_dim, action_dim)
        self.critic_target = TD3Critic(state_dim, action_dim)

        # Optimizers
        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=lr_actor)
        self.critic_optimizer = optim.Adam(self.critic.parameters(), lr=lr_critic)

        # Initialize target networks
        self.actor_target.load_state_dict(self.actor.state_dict())
        self.critic_target.load_state_dict(self.critic.state_dict())

    def select_action(self, state, add_noise=True):
        """Select action with target policy smoothing"""
        state_tensor = torch.FloatTensor(state).unsqueeze(0)
        action = self.actor(state_tensor).cpu().data.numpy().flatten()

        if add_noise:
            # Add clipped noise for exploration
            noise = np.random.normal(0, self.policy_noise, size=self.action_dim)
            noise = np.clip(noise, -self.noise_clip, self.noise_clip)
            action = action + noise
            action = np.clip(action, -self.max_action, self.max_action)

        return action

    def update(self, replay_buffer, batch_size=100):
        """Update TD3 networks"""
        self.total_it += 1

        # Sample batch from replay buffer
        state, action, next_state, reward, not_done = replay_buffer.sample(batch_size)

        with torch.no_grad():
            # Select action according to policy and add clipped noise
            noise = torch.FloatTensor(action).data.normal_(0, self.policy_noise)
            noise = noise.clamp(-self.noise_clip, self.noise_clip)

            next_action = (self.actor_target(next_state) + noise).clamp(
                -self.max_action, self.max_action
            )

            # Compute target Q-value
            target_Q1, target_Q2 = self.critic_target(next_state, next_action)
            target_Q = torch.min(target_Q1, target_Q2)
            target_Q = reward + not_done * self.gamma * target_Q

        # Get current Q estimates
        current_Q1, current_Q2 = self.critic(state, action)

        # Compute critic loss
        critic_loss = nn.MSELoss()(current_Q1, target_Q) + nn.MSELoss()(current_Q2, target_Q)

        # Optimize critic
        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        self.critic_optimizer.step()

        # Delayed policy updates
        if self.total_it % self.policy_freq == 0:
            # Compute actor loss
            actor_loss = -self.critic.Q1(state, self.actor(state)).mean()

            # Optimize actor
            self.actor_optimizer.zero_grad()
            actor_loss.backward()
            self.actor_optimizer.step()

            # Soft update target networks
            self.soft_update(self.actor_target, self.actor, self.tau)
            self.soft_update(self.critic_target, self.critic, self.tau)

    def soft_update(self, target_network, source_network, tau):
        """Soft update of target network parameters"""
        for target_param, param in zip(target_network.parameters(), source_network.parameters()):
            target_param.data.copy_(tau * param.data + (1 - tau) * target_param.data)
```

## Isaac Sim Integration for RL Training

### Creating RL Environments in Isaac Sim

```python
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from ommi.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.robots import Robot
from omni.isaac.core.objects import DynamicCuboid
import numpy as np
import torch

class IsaacRLEnvironment:
    def __init__(self, robot_usd_path, scene_config):
        # Initialize Isaac Sim world
        self.world = World(stage_units_in_meters=1.0)
        self.world.scene.add_default_ground_plane()

        # Add robot
        self.robot = self.world.scene.add(
            Robot(
                prim_path="/World/Robot",
                name="rl_robot",
                usd_path=robot_usd_path
            )
        )

        # Initialize environment parameters
        self.scene_config = scene_config
        self.max_episode_steps = 1000
        self.current_step = 0

        # Reset environment
        self.reset()

    def reset(self):
        """Reset the environment to initial state"""
        self.world.reset()
        self.current_step = 0

        # Reset robot to initial configuration
        initial_joint_positions = self.scene_config.get("initial_joint_positions", [0] * 7)
        self.robot.set_joint_positions(np.array(initial_joint_positions))

        # Reset objects in scene
        self.reset_objects()

        # Return initial observation
        return self.get_observation()

    def step(self, action):
        """Execute one step in the environment"""
        # Apply action to robot
        self.apply_action(action)

        # Step the simulation
        self.world.step(render=True)

        # Get next observation
        next_obs = self.get_observation()

        # Calculate reward
        reward = self.calculate_reward()

        # Check if episode is done
        done = self.is_episode_done()

        # Increment step counter
        self.current_step += 1

        return next_obs, reward, done, {}

    def get_observation(self):
        """Get current observation from environment"""
        # Get robot state
        joint_positions = self.robot.get_joint_positions()
        joint_velocities = self.robot.get_joint_velocities()

        # Get end-effector pose
        ee_pose = self.robot.get_end_effector_pose()

        # Get object information (if applicable)
        object_info = self.get_object_info()

        # Combine into observation vector
        observation = np.concatenate([
            joint_positions,
            joint_velocities,
            ee_pose.position,
            ee_pose.orientation,
            object_info
        ])

        return observation

    def apply_action(self, action):
        """Apply action to robot"""
        # Convert action to joint commands
        joint_commands = self.process_action(action)

        # Apply commands to robot
        self.robot.apply_action(joint_commands)

    def calculate_reward(self):
        """Calculate reward based on current state"""
        # Example reward function - customize based on task
        ee_pos = self.robot.get_end_effector_position()
        target_pos = self.scene_config.get("target_position", np.array([0, 0, 0]))

        # Distance to target
        distance_to_target = np.linalg.norm(ee_pos - target_pos)

        # Reward based on distance (negative distance)
        reward = -distance_to_target

        # Add bonus for reaching target
        if distance_to_target < 0.1:  # 10cm threshold
            reward += 100  # Large bonus for reaching target

        # Add penalty for joint limits
        joint_positions = self.robot.get_joint_positions()
        joint_limits = self.scene_config.get("joint_limits", [])
        if joint_limits:
            for i, (pos, limits) in enumerate(zip(joint_positions, joint_limits)):
                if pos < limits[0] or pos > limits[1]:
                    reward -= 10  # Penalty for exceeding joint limits

        return reward

    def is_episode_done(self):
        """Check if episode is done"""
        # Check step limit
        if self.current_step >= self.max_episode_steps:
            return True

        # Check for collisions (if applicable)
        if self.has_collision():
            return True

        # Check task completion
        if self.is_task_completed():
            return True

        return False

    def has_collision(self):
        """Check if robot has collided with environment"""
        # Implementation depends on collision detection setup
        return False

    def is_task_completed(self):
        """Check if task has been completed"""
        # Example: check if end-effector is close to target
        ee_pos = self.robot.get_end_effector_position()
        target_pos = self.scene_config.get("target_position", np.array([0, 0, 0]))
        distance = np.linalg.norm(ee_pos - target_pos)
        return distance < 0.05  # 5cm threshold
```

### Training Loop Implementation

```python
class RLTrainer:
    def __init__(self, agent, environment, max_episodes=1000, max_steps_per_episode=1000):
        self.agent = agent
        self.env = environment
        self.max_episodes = max_episodes
        self.max_steps_per_episode = max_steps_per_episode
        self.episode_rewards = []

    def train(self):
        """Main training loop"""
        for episode in range(self.max_episodes):
            # Reset environment for new episode
            state = self.env.reset()

            episode_reward = 0
            done = False

            for step in range(self.max_steps_per_episode):
                # Select action using current policy
                action = self.agent.select_action(state)

                # Execute action in environment
                next_state, reward, done, info = self.env.step(action)

                # Store experience in replay buffer
                self.agent.remember(state, action, reward, next_state, done)

                # Update agent
                self.agent.update()

                # Update current state
                state = next_state
                episode_reward += reward

                if done:
                    break

            # Log episode results
            self.episode_rewards.append(episode_reward)
            print(f"Episode {episode}: Reward = {episode_reward:.2f}, Steps = {step+1}")

            # Save model periodically
            if episode % 100 == 0:
                self.save_model(f"rl_model_episode_{episode}.pth")

    def save_model(self, filepath):
        """Save trained model"""
        torch.save({
            'actor_state_dict': self.agent.actor.state_dict(),
            'critic_state_dict': self.agent.critic.state_dict() if hasattr(self.agent, 'critic') else None,
            'episode_rewards': self.episode_rewards
        }, filepath)

    def load_model(self, filepath):
        """Load trained model"""
        checkpoint = torch.load(filepath)
        self.agent.actor.load_state_dict(checkpoint['actor_state_dict'])
        if checkpoint['critic_state_dict'] is not None:
            self.agent.critic.load_state_dict(checkpoint['critic_state_dict'])
        self.episode_rewards = checkpoint['episode_rewards']
```

## Advanced RL Techniques for Robotics

### Proximal Policy Optimization (PPO)

```python
import torch.nn as nn
import torch.optim as optim

class PPOActorCritic(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_dim=256):
        super(PPOActorCritic, self).__init__()

        # Shared feature extractor
        self.feature_extractor = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU()
        )

        # Actor head (for continuous actions)
        self.actor_mean = nn.Linear(hidden_dim, action_dim)
        self.actor_log_std = nn.Parameter(torch.zeros(action_dim))

        # Critic head
        self.critic = nn.Linear(hidden_dim, 1)

    def forward(self, state):
        features = self.feature_extractor(state)

        # Actor output
        action_mean = self.actor_mean(features)
        action_log_std = self.actor_log_std.expand_as(action_mean)

        # Critic output
        value = self.critic(features)

        return action_mean, action_log_std, value

    def get_action(self, state):
        """Sample action from policy"""
        action_mean, action_log_std, _ = self.forward(state)
        action_std = torch.exp(action_log_std)

        # Sample from normal distribution
        normal = torch.distributions.Normal(action_mean, action_std)
        action = normal.sample()
        log_prob = normal.log_prob(action).sum(dim=-1)

        return action, log_prob

    def evaluate(self, state, action):
        """Evaluate action probability and value"""
        action_mean, action_log_std, value = self.forward(state)
        action_std = torch.exp(action_log_std)

        normal = torch.distributions.Normal(action_mean, action_std)
        log_prob = normal.log_prob(action).sum(dim=-1)
        entropy = normal.entropy().sum(dim=-1)

        return log_prob, entropy, value

class PPOAgent:
    def __init__(self, state_dim, action_dim, lr=3e-4, gamma=0.99, eps_clip=0.2,
                 k_epochs=80, entropy_coef=0.01):
        self.lr = lr
        self.gamma = gamma
        self.eps_clip = eps_clip
        self.k_epochs = k_epochs
        self.entropy_coef = entropy_coef

        self.policy = PPOActorCritic(state_dim, action_dim)
        self.optimizer = optim.Adam(self.policy.parameters(), lr=lr)

        self.buffer = RolloutBuffer()

    def update(self):
        """Update policy using PPO"""
        # Monte Carlo estimate of state rewards
        rewards = []
        discounted_reward = 0
        for reward, is_terminal in zip(reversed(self.buffer.rewards), reversed(self.buffer.is_terminals)):
            if is_terminal:
                discounted_reward = 0
            discounted_reward = reward + (self.gamma * discounted_reward)
            rewards.insert(0, discounted_reward)

        # Normalizing the rewards
        rewards = torch.tensor(rewards, dtype=torch.float32)
        rewards = (rewards - rewards.mean()) / (rewards.std() + 1e-7)

        # Convert list to tensor
        old_states = torch.squeeze(torch.stack(self.buffer.states, dim=0)).detach()
        old_actions = torch.squeeze(torch.stack(self.buffer.actions, dim=0)).detach()
        old_logprobs = torch.squeeze(torch.stack(self.buffer.logprobs, dim=0)).detach()

        # Optimize policy for K epochs
        for _ in range(self.k_epochs):
            # Evaluating old actions and values
            logprobs, entropy, state_values = self.policy.evaluate(old_states, old_actions)

            # Finding the ratio (pi_theta / pi_theta__old)
            ratios = torch.exp(logprobs - old_logprobs.detach())

            # Finding Surrogate Loss
            advantages = rewards - state_values.detach()
            surr1 = ratios * advantages
            surr2 = torch.clamp(ratios, 1 - self.eps_clip, 1 + self.eps_clip) * advantages

            # Final loss of clipped objective PPO
            loss = -torch.min(surr1, surr2) + 0.5 * nn.MSELoss()(state_values, rewards) - self.entropy_coef * entropy

            # Take gradient step
            self.optimizer.zero_grad()
            loss.mean().backward()
            self.optimizer.step()

        # Clear buffer
        self.buffer.clear()

class RolloutBuffer:
    def __init__(self):
        self.actions = []
        self.states = []
        self.logprobs = []
        self.rewards = []
        self.state_values = []
        self.is_terminals = []

    def clear(self):
        del self.actions[:]
        del self.states[:]
        del self.logprobs[:]
        del self.rewards[:]
        del self.state_values[:]
        del self.is_terminals[:]
```

### Hindsight Experience Replay (HER)

```python
class HERBuffer:
    def __init__(self, buffer_size, k_future=4, reward_func=None):
        self.buffer_size = buffer_size
        self.k_future = k_future
        self.reward_func = reward_func
        self.buffer = deque(maxlen=buffer_size)

    def store_episode(self, episode_transitions):
        """Store an entire episode and generate HER transitions"""
        # Store original episode
        for transition in episode_transitions:
            self.buffer.append(transition)

        # Generate HER transitions
        episode_length = len(episode_transitions)

        for t in range(episode_length):
            # Get future time steps for goal relabeling
            future_timesteps = np.random.choice(
                range(t, episode_length),
                size=min(self.k_future, episode_length - t),
                replace=False
            )

            for future_time in future_timesteps:
                # Create new transition with relabeled goal
                transition = episode_transitions[t]
                future_transition = episode_transitions[future_time]

                # Relabel goal with achieved state from future
                new_goal = future_transition['achieved_state']

                # Calculate new reward with relabeled goal
                new_reward = self.reward_func(
                    transition['achieved_state'],
                    new_goal,
                    transition['action']
                )

                # Create HER transition
                her_transition = {
                    'state': transition['state'],
                    'action': transition['action'],
                    'reward': new_reward,
                    'next_state': transition['next_state'],
                    'done': transition['done'],
                    'goal': new_goal
                }

                self.buffer.append(her_transition)
```

## Domain Randomization and Sim-to-Real Transfer

### Domain Randomization in Isaac Sim

```python
class DomainRandomization:
    def __init__(self, environment):
        self.env = environment
        self.randomization_params = {
            'lighting': {'min': 0.5, 'max': 2.0},
            'textures': ['metal', 'wood', 'plastic'],
            'object_sizes': {'min': 0.8, 'max': 1.2},
            'friction': {'min': 0.1, 'max': 0.9},
            'mass': {'min': 0.5, 'max': 2.0}
        }

    def randomize_environment(self):
        """Randomize environment parameters"""
        # Randomize lighting conditions
        self.randomize_lighting()

        # Randomize object properties
        self.randomize_object_properties()

        # Randomize physics parameters
        self.randomize_physics()

        # Randomize sensor noise
        self.randomize_sensor_noise()

    def randomize_lighting(self):
        """Randomize lighting conditions"""
        # Get lighting prim
        light_prim = get_prim_at_path("/World/Light")

        if light_prim.IsValid():
            # Randomize intensity
            intensity = np.random.uniform(
                self.randomization_params['lighting']['min'],
                self.randomization_params['lighting']['max']
            ) * 1000  # Base intensity

            # Apply to light
            light_prim.GetAttribute("intensity").Set(intensity)

    def randomize_object_properties(self):
        """Randomize object properties"""
        # Get all objects in scene
        objects = self.env.get_all_objects()

        for obj in objects:
            # Randomize size
            size_factor = np.random.uniform(
                self.randomization_params['object_sizes']['min'],
                self.randomization_params['object_sizes']['max']
            )

            # Randomize material properties
            friction = np.random.uniform(
                self.randomization_params['friction']['min'],
                self.randomization_params['friction']['max']
            )

            # Apply randomizations
            obj.set_scale(size_factor)
            obj.set_friction(friction)

    def randomize_physics(self):
        """Randomize physics parameters"""
        # Randomize gravity
        gravity_magnitude = np.random.uniform(9.0, 10.0)
        self.env.set_gravity([0, -gravity_magnitude, 0])

        # Randomize time step
        timestep = np.random.uniform(0.001, 0.01)
        self.env.set_physics_timestep(timestep)

    def randomize_sensor_noise(self):
        """Randomize sensor noise parameters"""
        # Add random noise to sensor readings
        self.env.camera.add_noise(
            mean=0,
            std=np.random.uniform(0.001, 0.01)
        )
```

## Multi-Task and Transfer Learning

### Multi-Task RL Agent

```python
class MultiTaskAgent:
    def __init__(self, state_dim, action_dim, num_tasks, hidden_dim=256):
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.num_tasks = num_tasks

        # Shared feature extractor
        self.feature_extractor = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU()
        )

        # Task-specific heads
        self.task_heads = nn.ModuleList([
            nn.Sequential(
                nn.Linear(hidden_dim, hidden_dim),
                nn.ReLU(),
                nn.Linear(hidden_dim, action_dim)
            ) for _ in range(num_tasks)
        ])

        # Task classifier (for unsupervised task identification)
        self.task_classifier = nn.Linear(hidden_dim, num_tasks)

        self.optimizer = optim.Adam(list(self.feature_extractor.parameters()) +
                                   list(self.task_heads.parameters()) +
                                   list(self.task_classifier.parameters()))

    def forward(self, state, task_id=None):
        """Forward pass with optional task specification"""
        features = self.feature_extractor(state)

        if task_id is not None:
            # Use specific task head
            action = self.task_heads[task_id](features)
        else:
            # Use all task heads and select based on classification
            task_probs = torch.softmax(self.task_classifier(features), dim=-1)
            actions = torch.stack([head(features) for head in self.task_heads], dim=1)
            action = torch.sum(actions * task_probs.unsqueeze(-1), dim=1)

        return action
```

## Practical RL Applications in Robotics

### Navigation with RL

```python
class RLNavigationAgent:
    def __init__(self, state_dim, action_dim, max_linear_vel=1.0, max_angular_vel=1.0):
        self.action_dim = action_dim
        self.max_linear_vel = max_linear_vel
        self.max_angular_vel = max_angular_vel

        # Use DDPG for continuous navigation control
        self.agent = DDPGAgent(
            state_dim=state_dim,
            action_dim=action_dim,
            max_action=1.0
        )

    def get_navigation_action(self, observation):
        """Get navigation action from RL agent"""
        # Process observation (laser scan, goal direction, etc.)
        processed_obs = self.process_navigation_observation(observation)

        # Get action from agent
        action = self.agent.select_action(processed_obs)

        # Convert to velocity commands
        linear_vel = action[0] * self.max_linear_vel
        angular_vel = action[1] * self.max_angular_vel

        return linear_vel, angular_vel

    def process_navigation_observation(self, raw_obs):
        """Process raw navigation observation"""
        # Example: laser scan + goal direction + robot pose
        laser_scan = raw_obs['laser_scan']
        goal_direction = raw_obs['goal_direction']
        robot_pose = raw_obs['robot_pose']

        # Normalize laser scan
        laser_scan = np.clip(laser_scan / 10.0, 0, 1)  # Assuming max range 10m

        # Combine all observations
        processed_obs = np.concatenate([laser_scan, goal_direction, robot_pose])

        return processed_obs
```

### Manipulation with RL

```python
class RLManipulationAgent:
    def __init__(self, robot_state_dim, action_dim, max_joint_vel=1.0):
        self.max_joint_vel = max_joint_vel

        # Use TD3 for manipulation control
        self.agent = TD3Agent(
            state_dim=robot_state_dim,
            action_dim=action_dim,
            max_action=1.0
        )

    def get_manipulation_action(self, observation):
        """Get manipulation action from RL agent"""
        # Process observation (joint states, end-effector pose, object pose)
        processed_obs = self.process_manipulation_observation(observation)

        # Get action from agent
        action = self.agent.select_action(processed_obs)

        # Convert to joint velocity commands
        joint_velocities = action * self.max_joint_vel

        return joint_velocities

    def process_manipulation_observation(self, raw_obs):
        """Process raw manipulation observation"""
        # Example: joint positions, velocities, end-effector pose, object pose
        joint_positions = raw_obs['joint_positions']
        joint_velocities = raw_obs['joint_velocities']
        ee_pose = raw_obs['end_effector_pose']
        object_pose = raw_obs['object_pose']

        # Normalize joint positions to [-1, 1]
        normalized_joints = np.tanh(joint_positions)  # Assuming joint limits

        # Combine all observations
        processed_obs = np.concatenate([
            normalized_joints,
            joint_velocities,
            ee_pose,
            object_pose
        ])

        return processed_obs
```

## Training Best Practices

### Curriculum Learning

```python
class CurriculumLearner:
    def __init__(self, tasks, success_threshold=0.8):
        self.tasks = tasks  # Ordered list of tasks from easy to hard
        self.success_threshold = success_threshold
        self.current_task_idx = 0
        self.task_performance = [0.0] * len(tasks)

    def update_task_performance(self, task_idx, success):
        """Update performance for a task"""
        # Simple moving average of success rate
        self.task_performance[task_idx] = 0.9 * self.task_performance[task_idx] + 0.1 * success

    def should_advance_task(self):
        """Check if we should advance to the next task"""
        current_performance = self.task_performance[self.current_task_idx]
        return current_performance >= self.success_threshold

    def advance_task(self):
        """Advance to the next task in curriculum"""
        if self.current_task_idx < len(self.tasks) - 1 and self.should_advance_task():
            self.current_task_idx += 1
            print(f"Advancing to task {self.current_task_idx + 1}: {self.tasks[self.current_task_idx]}")

    def get_current_task(self):
        """Get the current task configuration"""
        return self.tasks[self.current_task_idx]
```

### Reward Shaping

```python
class ShapedRewardFunction:
    def __init__(self, task_config):
        self.task_config = task_config
        self.weights = task_config.get('reward_weights', {
            'progress': 1.0,
            'efficiency': 0.1,
            'safety': 0.5,
            'completion': 10.0
        })

    def calculate_reward(self, state, action, next_state, done):
        """Calculate shaped reward"""
        reward = 0.0

        # Progress reward - encourage movement toward goal
        progress_reward = self.calculate_progress_reward(state, next_state)
        reward += self.weights['progress'] * progress_reward

        # Efficiency reward - penalize unnecessary movements
        efficiency_reward = self.calculate_efficiency_reward(action)
        reward += self.weights['efficiency'] * efficiency_reward

        # Safety reward - penalize dangerous states
        safety_reward = self.calculate_safety_reward(next_state)
        reward += self.weights['safety'] * safety_reward

        # Completion reward - large reward for task completion
        if done and self.is_task_successful(next_state):
            reward += self.weights['completion']

        return reward

    def calculate_progress_reward(self, state, next_state):
        """Calculate reward based on progress toward goal"""
        # Example: distance to goal
        current_dist = self.get_distance_to_goal(state)
        next_dist = self.get_distance_to_goal(next_state)

        # Positive if we moved closer to goal
        return max(0, current_dist - next_dist)

    def calculate_efficiency_reward(self, action):
        """Calculate reward based on action efficiency"""
        # Penalize large actions to encourage smooth movements
        action_norm = np.linalg.norm(action)
        return -0.01 * action_norm  # Small penalty for large actions

    def calculate_safety_reward(self, state):
        """Calculate reward based on safety"""
        # Penalize being too close to obstacles
        min_distance = self.get_min_obstacle_distance(state)
        if min_distance < 0.2:  # 20cm threshold
            return -1.0
        return 0.0
```

## Troubleshooting and Debugging

### Common RL Issues and Solutions

```python
class RLDiagnostic:
    def __init__(self, agent):
        self.agent = agent
        self.metrics = {
            'rewards': [],
            'losses': [],
            'exploration': [],
            'success_rate': []
        }

    def diagnose_training(self):
        """Diagnose common RL training issues"""
        issues = []

        # Check for reward stagnation
        if self.is_reward_stagnating():
            issues.append("Reward stagnation detected - consider reward reshaping or exploration adjustment")

        # Check for loss instability
        if self.is_loss_unstable():
            issues.append("Loss instability detected - consider learning rate reduction or gradient clipping")

        # Check for poor exploration
        if self.is_exploration_poor():
            issues.append("Poor exploration detected - consider increasing exploration noise")

        return issues

    def is_reward_stagnating(self, window=100):
        """Check if rewards are stagnating"""
        if len(self.metrics['rewards']) < window:
            return False

        recent_rewards = self.metrics['rewards'][-window:]
        return np.std(recent_rewards) < 0.01  # Very low variance indicates stagnation

    def is_loss_unstable(self, threshold=10.0):
        """Check if losses are unstable"""
        if not self.metrics['losses']:
            return False

        return np.any(np.abs(self.metrics['losses'][-10:]) > threshold)

    def is_exploration_poor(self, threshold=0.1):
        """Check if exploration is poor"""
        if not self.metrics['exploration']:
            return True

        return np.mean(self.metrics['exploration'][-50:]) < threshold
```

## Best Practices for RL in Robotics

### 1. Simulation Design
- Create diverse simulation environments
- Implement proper domain randomization
- Ensure simulation fidelity matches reality
- Include realistic sensor noise models

### 2. Reward Engineering
- Design sparse rewards for final goals
- Add shaped rewards for intermediate progress
- Balance different reward components
- Test reward function thoroughly

### 3. Exploration Strategies
- Use appropriate exploration noise
- Implement curriculum learning
- Consider intrinsic motivation
- Monitor exploration statistics

### 4. Safety Considerations
- Implement action clipping and limits
- Add safety constraints to reward
- Use simulation for dangerous tasks
- Plan for graceful failure

## Summary

Reinforcement Learning provides a powerful framework for learning complex robotic behaviors directly from environmental interaction. The NVIDIA Isaac Platform, combined with Isaac Sim, offers an ideal environment for training RL agents with high-fidelity simulation and synthetic data generation.

Key aspects of successful RL implementation in robotics include:
- Proper choice of RL algorithm based on action space
- Effective reward function design
- Domain randomization for sim-to-real transfer
- Curriculum learning for complex tasks
- Proper safety considerations and constraints

In the next section, we'll explore sim-to-real transfer techniques that enable successful deployment of simulation-trained robots to real-world applications.