import os
import time
import kthread
import threading
import env_vars
import csv
import random
import ast
import helpers
import torch as T
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import DataLoader, Dataset 
import torch.optim as optim
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from datetime import datetime
from sklearn import preprocessing
import copy
from tqdm import tqdm

# lstm to train regression problem
class Model(nn.Module):
    def __init__(self, lr, input_dims, output_dims) -> None:
        super(Model, self).__init__()
        self.lr = lr
        self.input_dims = input_dims
        self.output_dims = output_dims

        self.fc1 = nn.Linear(self.input_dims, 64)
        self.fc2 = nn.Linear(64, 64)
        self.fc3 = nn.Linear(64, 32)
        self.fc4 = nn.Linear(32, self.output_dims)

        self.optimizer = optim.Adam(self.parameters(), lr = self.lr)
        self.device = T.device('cuda:0' if T.cuda.is_available() else 'cpu')

        self.to(self.device)

        self.fc1.weight.data.normal_(0, 1e-1)
        self.fc2.weight.data.normal_(0, 1e-1)
        self.fc3.weight.data.normal_(0, 1e-2)
        self.fc1.bias.data.normal_(0, 1e-1)
        self.fc2.bias.data.normal_(0, 1e-1)
        self.fc3.bias.data.normal_(0, 1e-2)

    def forward(self, state):

        fc1_out = nn.functional.sigmoid(self.fc1(state))
        fc2_out = nn.functional.sigmoid(self.fc2(fc1_out))
        fc3_out = nn.functional.sigmoid(self.fc3(fc2_out))
        fc4_out = nn.functional.sigmoid(self.fc4(fc3_out))

        return fc4_out

    def save_checkpoint(self, chkpt_path):
        T.save(self.state_dict(), chkpt_path)

    def load_checkpoint(self, chkpt_path):
        self.load_state_dict(T.load(chkpt_path))

class GAGenerator:
    def __init__(self, scenario, k, top_k_configs, configs):
        # self.dataset = dataset    
        # self.dataloader = DataLoader(dataset, batch_size=1, shuffle=True)
        self.scenario = scenario    
        self.k = k   
        self.top_k_configs = top_k_configs
        self.configs = configs      

        self.F = .4
        self.CR = .5

        self.model = Model(
            0.0005, 
            len(env_vars.simulation_vars[scenario]['state_cols']) + len(env_vars.simulation_vars[scenario]['modes_vec']) + k, 
            1) 
        self.model.load_checkpoint(env_vars.MODELS + scenario + '_' + str(k) + 'params')
        self.model.eval()

    def normalize_state(self, state):
        _state = {}
        i = 0
        try:
            for p in state:
                if 'MODE' not in p:
                    min_val = float(env_vars.state_ranges[p]['min'])
                    max_val = float(env_vars.state_ranges[p]['max'])

                    if state[p] > max_val:
                        state[p] = max_val 
                    if state[p] < min_val:
                        state[p] = min_val 
                    
                    _state[p] = ((state[p] - min_val) / (max_val - min_val))
                else:
                    _state[p] = state[p]
                i += 1
        except Exception as e:
            raise Exception("State normalization exception: " + str(e))
        return T.tensor([_state[i] for i in sorted(state, key=str.lower)])

    def normalize_param(self, param, value):
        min_val = float(self.configs[param]['range'][0])
        max_val = float(self.configs[param]['range'][1])
        return ((value - min_val) / (max_val - min_val))

    def initialize_randomized_params(self):
        params = []
        i = 0
        for p in self.top_k_configs:
            _r = random.uniform(float(self.configs[p['name']]['range'][0]), float(self.configs[p['name']]['range'][1]))
            if self.configs[p['name']]['increment'] is not None:
                r = float(self.configs[p['name']]['range'][0])
                while r < _r:
                    r += self.configs[p['name']]['increment']
                params.append(round(r, 2))
            else:
                params.append(_r)
        return params
    
    def rescale_params(self, param, value):
        old_min = 0
        old_max = 1
        new_min = float(self.configs[param]['range'][0])
        new_max = float(self.configs[param]['range'][1])
        rescaled_val = ( ( value - old_min ) / (old_max - old_min) ) * (new_max - new_min) + new_min
        _rescaled_val = float(self.configs[param]['range'][0])
        if self.configs[param]['increment'] is not None:
            while _rescaled_val < rescaled_val:
                _rescaled_val += self.configs[param]['increment']
            return round(_rescaled_val, 2)    
        return rescaled_val
        
    def generate(self, state):
        state = T.tensor(state, dtype=T.float)
        population = []

        print('Initializing')
        # for i in tqdm(range(100)):
        for i in range(100):
            init = self.initialize_randomized_params()
            to_append = []
            for j in range(len(init)):
                to_append.append(self.normalize_param(self.top_k_configs[j]['name'], init[j]))
            
            population.append(T.tensor(to_append))

        # initial scores populate
        scores = []

        # calculate state dev
        for conf in population:
            _x = T.concat([state, conf], dim=0)
            y_hat = self.model(_x)
            scores.append({
                'x': state,
                'conf': conf,
                '_x': _x,
                'fitness': y_hat.item()
            })

        print('Mutating')
        # for i in tqdm(range(100)):
        for i in range(100):

            # select top deviator
            sorted_scores = sorted(scores, key=lambda x: x['fitness'], reverse=True)
            best = sorted_scores[0]

            #mutate
            _scores = []
            for idx, chromo in enumerate(scores):

                _x = T.concat([state, conf], dim=0)
                y_hat = self.model(_x)
                _scores.append({
                    'x': state,
                    'conf': conf,
                    '_x': _x,
                    'fitness': y_hat.item()
                })

                #get 2 other random
                r1 = scores[random.randint(0, len(scores)-1)]
                r2 = scores[random.randint(0, len(scores)-1)]
                r_diff = r1['conf']-r2['conf']
                b_diff = best['conf'] - chromo['conf']

                _scores.append({
                    'x': state,
                    '_x': _x,
                    'conf': chromo['conf'] + self.F*b_diff + self.F*r_diff,
                    'fitness': y_hat.item()
                })

            #crossover
            __scores = []
            for idx, s in enumerate(scores):
                if random.random() < self.CR:
                    __scores.append(_scores[idx])
                else:
                    __scores.append(scores[idx]) 

            try:
                # calculate state dev
                for idx, chromo in enumerate(scores):

                    if scores[idx]['fitness'] < __scores[idx]['fitness']:
                        scores[idx]['fitness'] = __scores[idx]['fitness']

            except Exception as e:
                print(e)

        return sorted(scores, key=lambda x: x['fitness'], reverse=True)[0]
