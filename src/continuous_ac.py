import torch as T
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import helpers
import env_vars as env_vars
import random
import numpy as np

class Memory:
    def __init__(self, buffer_size, batch_size):
        self.buffer_size = buffer_size
        self.batch_size = batch_size
        self.memory = []
        self.position = 0

    def push(self, element):
        
        if len(self.memory) < self.buffer_size:
            self.memory.append(None)
        self.memory[self.position] = element
        self.position = (self.position + 1) % self.buffer_size

    def sample(self):
        return list(zip(*random.sample(self.memory, self.batch_size)))

    def __len__(self):
        return len(self.memory)
    
class ActorNetwork(nn.Module):
    def __init__(self, lr, input_dims, n_configs, fc1_dims, fc2_dims) -> None:
        super(ActorNetwork, self).__init__()
        self.lr = lr
        self.input_dims = input_dims
        self.fc1_dims = fc1_dims
        self.fc2_dims = fc2_dims
        self.n_actions = n_configs

        self.lstm = nn.LSTM(*input_dims, self.fc2_dims, num_layers = 2, batch_first=True)

        self.bn = nn.BatchNorm1d(self.fc1_dims)
        self.fc1 = nn.Linear(*self.input_dims, self.fc1_dims)
        self.fc2 = nn.Linear(self.fc1_dims, self.fc2_dims)
        self.mu = nn.Linear(fc2_dims, self.n_actions)
        self.sigma = nn.Linear(fc2_dims, self.n_actions)
        self.reset_parameters()

        self.optimizer = optim.Adam(self.parameters(), lr = self.lr)
        self.device = T.device('cuda:0' if T.cuda.is_available() else 'cpu')

        self.to(self.device)
        
    def reset_parameters(self):
        self.fc1.weight.data.normal_(0, 1e-1)
        self.fc2.weight.data.normal_(0, 1e-1)
        self.mu.weight.data.normal_(0, 1e-2)
        self.sigma.weight.data.normal_(0, 1e-2)
        self.fc1.bias.data.normal_(0, 1e-1)
        self.fc2.bias.data.normal_(0, 1e-1)
        self.mu.bias.data.normal_(0, 1e-2)
        self.sigma.bias.data.normal_(0, 1e-2)

    def forward_seq(self, state_seq):
        # normalize state values
        # ss_normed = state_seq / state_seq.max(0, keepdim=True)[0]

        # state_seq = T.tensor([state_seq], dtype=T.float).to(self.device)
        # actor_output_raw = nn.functional.tanh(self.fc2(nn.functional.tanh(self.fc1(state))))

        # mu_values = T.sigmoid(self.mu(actor_output_raw)) # actor's interpretation of state + chosen param type
        # sigma_values = T.sigmoid(self.sigma(actor_output_raw)) # actor's interpretation of state + chosen param type

        lstm_out, _ = self.lstm(state_seq)
        # lstm_out, _ = self.lstm(ss_normed)
        lstm_act = nn.functional.tanh(lstm_out)

        mu_values = T.sigmoid(self.mu(lstm_act)) # actor's interpretation of state + chosen param type
        sigma_values = T.sigmoid(self.sigma(lstm_act)) # actor's interpretation of state + chosen param type
                
        # return mu_values[-1], sigma_values[-1]
        last_seq_mu = []
        last_seq_sd = []
        for seq in mu_values:
            last_seq_mu.append(seq[-1])
        for seq in sigma_values:
            last_seq_sd.append(seq[-1])
        last_seq_mu = T.stack(last_seq_mu)
        last_seq_sd = T.stack(last_seq_sd)
        # last_seq_mu = T.unsqueeze(T.tensor(last_seq_mu, dtype=T.float), dim=1)
        # last_seq_sd = T.unsqueeze(T.tensor(last_seq_sd, dtype=T.float), dim=1)
        return last_seq_mu, last_seq_sd

    def forward(self, state):

        fc1_out = T.sigmoid((self.fc1(state)))
        fc2_out = T.sigmoid(self.fc2(fc1_out))

        mu_values = T.sigmoid(self.mu(fc2_out))
        sigma_values = T.sigmoid(self.sigma(fc2_out))

        return mu_values, sigma_values
    
    def save_checkpoint(self, chkpt_path):
        T.save(self.state_dict(), chkpt_path)

    def load_checkpoint(self, chkpt_path):
        self.load_state_dict(T.load(chkpt_path))

class CriticNetwork(nn.Module):
    def __init__(self, lr, input_dims, n_configs, fc1_dims, fc2_dims) -> None:
        super(CriticNetwork, self).__init__()
        self.lr = lr
        self.input_dims = input_dims
        self.fc1_dims = fc1_dims
        self.fc2_dims = fc2_dims
        
        # self.lstm = nn.LSTM(*input_dims, 128, num_layers = 2, batch_first = True)
        # self.fc1 = nn.Linear(128, 32)
        # self.fc2 = nn.Linear(32+n_configs, 32)
        # self.c_val = nn.Linear(32, 1)
        
        self.bn = nn.BatchNorm1d(self.fc1_dims)
        self.fc1 = nn.Linear(input_dims[0]+n_configs, self.fc1_dims)
        self.fc2 = nn.Linear(self.fc1_dims, self.fc2_dims)
        self.c_val = nn.Linear(self.fc2_dims, 1)
        self.reset_parameters()

        self.optimizer = optim.Adam(self.parameters(), lr = self.lr)
        self.device = T.device('cuda:0' if T.cuda.is_available() else 'cpu')

        self.to(self.device)

    def forward_seq(self, state_seq, actions):
        state_seq = T.tensor(state_seq, dtype=T.float).to(self.device)

        # normalize state values
        # ss_normed = state_seq / state_seq.max(0, keepdim=True)[0]

        # return self.critic(state)
        lstm_val, _ = self.lstm(state_seq)
        # lstm_val, _ = self.lstm(ss_normed)

        last_seq = []
        for seq in lstm_val:
            last_seq.append(seq[-1])
        last_seq = T.stack(last_seq)

        fc1_out = nn.functional.relu(self.fc1(nn.functional.relu(last_seq)))

        actions = T.tensor(actions, dtype=T.float).to(self.device)
        catted = T.concat((fc1_out, actions), dim=1)

        fc2_out = nn.functional.relu(self.fc2(catted))

        critic_val = nn.functional.tanh(self.c_val(fc2_out))

        return critic_val
        # return nn.functional.tanh(self.c_val(nn.functional.tanh(self.lstm(state_seq))))

    def forward(self, state, actions):
        state = T.tensor(state, dtype=T.float).to(self.device)
        actions = T.tensor(actions, dtype=T.float).to(self.device)
        catted = T.concat((state, actions), dim=1)

        fc1_out = T.sigmoid((self.fc1(catted)))
        fc2_out = T.sigmoid(self.fc2(fc1_out))
        critic_val = nn.functional.tanh(self.c_val(fc2_out))

        return critic_val
        
    def reset_parameters(self):
        self.fc1.weight.data.normal_(0, 1e-1)
        self.fc2.weight.data.normal_(0, 1e-1)
        self.c_val.weight.data.normal_(0, 1e-2)
        self.fc1.bias.data.normal_(0, 1e-1)
        self.fc2.bias.data.normal_(0, 1e-1)
        self.c_val.bias.data.normal_(0, 1e-2)

    def save_checkpoint(self, chkpt_path):
        T.save(self.state_dict(), chkpt_path)

    def load_checkpoint(self, chkpt_path):
        self.load_state_dict(T.load(chkpt_path))
    
class Agent(object):
    def __init__(self, input_dims, n_configs, memory, helper, n_epochs=3,
        alpha=0.00025, beta=0.0005, gamma=.99,
        layer1_size=128, layer2_size=64) -> None:

        self.gamma = gamma
        self.tau = 1e-2
        self.log_probs = None
        self.n_configs = n_configs
        self.actor = ActorNetwork(alpha, input_dims, n_configs, layer1_size, layer2_size)
        self.critic = CriticNetwork(beta, input_dims, n_configs, layer1_size, layer2_size)
        self.actor_target = ActorNetwork(alpha, input_dims, n_configs, layer1_size, layer2_size)
        self.critic_target = CriticNetwork(beta, input_dims, n_configs, layer1_size, layer2_size)
        self.actor_optimizer = T.optim.Adam(self.actor.parameters(), lr=1e-3)
        self.critic_optimizer = T.optim.Adam(self.critic.parameters(), lr=1e-3)
        self.helper: helpers.Helper = helper
        self.n_epochs = n_epochs
        self.memory = memory
        
        # copy weights
        self.hard_update(self.actor_target, self.actor)
        self.hard_update(self.critic_target, self.critic)
        
    def hard_update(self, target, network):
        for target_param, param in zip(target.parameters(), network.parameters()):
            target_param.data.copy_(param.data)
            
    def soft_update(self, target, network):
        for target_param, param in zip(target.parameters(), network.parameters()):
            target_param.data.copy_(self.tau*param.data + (1-self.tau)*target_param.data)
        
    def choose_params(self, state):
        # for individual datapoints
        state_b = T.tensor(state, dtype=T.float).to(self.actor.device)

        mus, sigmas  = self.actor(state_b)
        sigmas += T.full((self.n_configs,), 1e-3) # ensure sigmas are non-zero
        distributions =  T.distributions.Normal(mus, sigmas) # create an array of distributions using means and standard deviations
        values_sampled = distributions.sample() # sample the values from the distributions

        #add noise
        noise = T.full((self.n_configs,), np.random.normal(0, 0.05))
        values_with_noise = T.clamp(values_sampled + noise, min=0, max=1)
        
        self.helper.learn_log("Chosen values : " + str(values_with_noise))

        return values_with_noise
    
    def learn_batch(self, batch):

            #  Sample from memory
            state, action, reward, next_state = batch

            state = T.tensor(state).float()
            next_state = T.tensor(next_state).float()
            reward = T.tensor(reward).float()
            
            if len(action[0].shape) == 0:
                unsqueezed = np.expand_dims(action, axis=1)
                action = T.tensor(unsqueezed).float()
            else:
                action = T.tensor(action).float()
            
            # update critic
            mu_next, sigma_next = self.actor_target(next_state)
            sigma_next += T.full((self.n_configs,), 1e-3) # ensure sigmas are non-zero
            distributions_next =  T.distributions.Normal(mu_next, sigma_next) # create an array of distributions using means and standard deviations
            next_action = distributions_next.sample() # sample the values from the distributions
            
            Q_target = self.critic_target(next_state, next_action).detach()
            Q_target = reward.unsqueeze(1) + self.gamma*Q_target

            Q_estimate = self.critic(state, action)
            critic_loss = F.mse_loss(Q_estimate, Q_target)      
            self.helper.learn_log("Critic loss : " + str(critic_loss))  
            
            self.critic_optimizer.zero_grad()
            critic_loss.backward()
            self.critic_optimizer.step()
            
            # update actor
            mu, sigma = self.actor(state)
            sigma += T.full((self.n_configs,), 1e-3) # ensure sigmas are non-zero
            distributions =  T.distributions.Normal(mu, sigma) # create an array of distributions using means and standard deviations
            action = distributions.sample() # sample the values from the distributions
            
            actor_loss = -self.critic(state, action).mean()
            self.helper.learn_log("Actor loss : " + str(actor_loss))
            
            self.actor_optimizer.zero_grad()
            actor_loss.backward()
            self.actor_optimizer.step()
        
            # update actor_target and critic_target
            
            self.soft_update(self.critic_target, self.critic)
            self.soft_update(self.actor_target, self.actor)

