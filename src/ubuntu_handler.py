import os
import csv
import time
import math
import kthread
import threading
import helpers
import env_vars
import random
import numpy as np
import pandas as pd
import ast
from copy import deepcopy
import matplotlib.pyplot as plt
from pymavlink_simulation import Simulation
from subprocess import *
from datetime import datetime
from continuous_ac import Agent, Memory
import ga_generator as ga_generator
# from proximal_policy_optimization import Agent

class Handler:

    def __init__(self, curr_scenario, algo, curr_session=None, ranking=False, k=1):

        self.qground_process: Popen = None
        self.gazebo_process: Popen = None
        self.ardu_process: Popen = None
        
        self.sim_threads = []
        self.sim_thread: kthread.KThread = None
        self.sim_instance: Simulation = None
        
        self.set_threads = []
        self.set_thread: kthread.KThread = None
        self.started_setting = False

        self.autopilot_ready_time = None
        self.end = False
        self.started_simulation = False

        self.policies = []
        self.current_scenario = curr_scenario
        self.helper = helpers.Helper(curr_scenario)

        if curr_session is not None:
            self.session = curr_session
        else:
            self.session = str(time.time()).replace('.', '_')
        self.helper.current_session = self.session
        self.helper.new_session()
        self.start_time = time.time()

        self.helper.debug_log("Fuzzing scenario : " + self.current_scenario)
        self.violation_scores = deepcopy(env_vars.violation_scores_blank)
        self.vio_timestamps = []
        self.algo = algo

        self.memory = Memory(1000000, 32)

        # load memory
        try:
            if self.algo == 'drl' and curr_session is not None:
                episodes = [d for d in os.listdir(env_vars.ARDU_LOGS + self.current_scenario + '/' + curr_session) if os.path.isdir(env_vars.ARDU_LOGS + self.current_scenario + '/' + curr_session + '/' + d)]
                if len(episodes) > 0:
                    latest_episode = max(['.'.join(m.split('_')) for m in episodes]).replace('.', '_')
                    latest_memory = np.load(env_vars.ARDU_LOGS + self.current_scenario + '/' + curr_session + '/' + latest_episode + '/memory_' + latest_episode, allow_pickle=True)
                    self.memory.memory = latest_memory.tolist()
                    self.memory.position = len(self.memory) % self.memory.buffer_size
                    self.helper.debug_log("Loaded saved memory : " + latest_episode)
        except Exception as e:
            self.helper.debug_log("Memory load exception: " + str(e))

        # configs
        self.configs = None

        with open(os.path.join(env_vars.PROJ_HOME, 'src/VSM/parser/output_copter.csv')) as csv_file:
            self.configs = {}
            file_data = csv.reader(csv_file, delimiter=',')
            headers = next(file_data)
            for row in file_data:
                row_dict = dict(zip(headers, row))

                self.configs[row[0]] = {
                    'name': row_dict['Name'],
                    'description': row_dict['Description'],
                    'range': [] if row_dict['Range'] == '' else  row_dict['Range'].split(' '),
                    'increment': None if row_dict['Increment'] == '' else float(row_dict['Increment']), 
                    'value': [] if row_dict['Value'] == '' else  [v for v in row_dict['Value'].split('|') if v != '']
                }
            print("Loaded configuration params")
            self.helper.debug_log("Loaded configuration params")
        try:
            self.configs_rankings = pd.read_csv(env_vars.PROJ_HOME + 'src/VSM/rankings_config.csv').to_dict()
            if 'Unnamed: 0' in self.configs_rankings: self.configs_rankings.pop('Unnamed: 0')
            print("Loaded Config Rankings")
            self.helper.debug_log("Loaded Config Rankings")

        except Exception as e:
            self.helper.debug_log("src/VSM/rankings_config.csv loading error")

        self.top_k_configs = []
        self.ranking = ranking
        self.k = k
        if self.ranking:

            rankings_for_curr_pol = list(self.configs_rankings[self.current_scenario].keys())
            self.tupled_cfgs = [self.configs_rankings[self.current_scenario][i] for i in rankings_for_curr_pol]
            self.tupled_cfgs = [ast.literal_eval(cfg) for cfg in self.tupled_cfgs]
            self.tupled_cfgs = [cfg for cfg in self.tupled_cfgs if cfg[0] not in env_vars.params_to_exclude]

            i = 0
            j = 0
            while i < k:
                if len(self.configs[self.tupled_cfgs[j][0]]['range']) != 0:
                    self.top_k_configs.append(self.configs[self.tupled_cfgs[j][0]])
                    self.top_k_configs[i]['cosine_dist'] = self.tupled_cfgs[j][1]
                    i += 1 
                j += 1
            self.helper.debug_log("Choosen configs: " + ', '.join([self.top_k_configs[i]['name'] for i in  range(k)]))

        else:
            for c in env_vars.simulation_vars[self.current_scenario]['params_to_fuzz']:
                self.configs[c]['default'] = env_vars.param_defaults[c]
                self.top_k_configs.append(self.configs[c])
            self.helper.debug_log("Choosen configs: " + ', '.join([self.top_k_configs[i]['name'] for i in  range(len(env_vars.simulation_vars[self.current_scenario]['params_to_fuzz']))]))

        if self.algo == 'drl':

            # create Agent 
            if self.ranking:
                self.ac_agent = Agent(
                    [len(env_vars.simulation_vars[self.current_scenario]['state_cols'])+len(env_vars.simulation_vars[self.current_scenario]['modes_vec'])], 
                    self.k, self.memory, self.helper)
            else:
                self.ac_agent = Agent(
                    [len(env_vars.simulation_vars[self.current_scenario]['state_cols'])+len(env_vars.simulation_vars[self.current_scenario]['modes_vec'])], 
                    len(env_vars.simulation_vars[self.current_scenario]['params_to_fuzz']), self.memory, self.helper)
            
            # load latest saved model
            try:
                if curr_session is not None:
                    episodes = [d for d in os.listdir(env_vars.ARDU_LOGS + self.current_scenario + '/' + curr_session) if os.path.isdir(env_vars.ARDU_LOGS + self.current_scenario + '/' + curr_session + '/' + d)]
                    if len(episodes) > 0:
                        latest_save = max(['.'.join(m.split('_')) for m in episodes]).replace('.', '_')
                        path = env_vars.ARDU_LOGS + self.current_scenario + '/' + curr_session + '/' + latest_save + '/' + latest_save
                        self.ac_agent.actor.load_checkpoint(path + "_actor")
                        self.ac_agent.critic.load_checkpoint(path + "_critic")
                        self.helper.debug_log("Loaded saved model : " + latest_save)
            except Exception as e:
                self.helper.debug_log("Model load exception: " + str(e))

        elif self.algo == 'ga':

            # load generator
            self.ga_generator = ga_generator.GAGenerator(self.current_scenario, self.k, self.top_k_configs, self.configs)

    def start_simulation(self): 

        self.wait()

        try:
            self.qground_process = self.run_qground()
            self.gazebo_process = self.run_gazebo()
            self.ardu_process = self.run_ardu()
            self.autopilot_ready_time = time.time()
        except Exception as e:
            self.helper.debug_log("Simulator Startup Exception")

        self.loop()

    def run_qground(self, c='gnome-terminal -- ./QGroundControl.AppImage', d=env_vars.PROJ_HOME+'bins/'): 
        self.helper.debug_log("Starting QGCS")
        return Popen(c, cwd = d, shell=True)

    def run_gazebo(self, c='gnome-terminal -- gzserver --verbose worlds/iris_arducopter_demo.world', d=None):
    # def run_gazebo(self, c='gnome-terminal -- gazebo --verbose worlds/iris_arducopter_demo.world', d=None):
        self.helper.debug_log("Starting Gazebo")
        return Popen(c, cwd = d, shell=True)

    def run_ardu(self, c='gnome-terminal -- ../Tools/autotest/sim_vehicle.py -f gazebo-iris', d=env_vars.ARDU_HOME + 'ArduCopter/'):
        self.helper.debug_log("Starting ArduPilot")
        return Popen(c, cwd = d, shell=True)
    
    def loop(self, buffer=7):
        
        threshold=env_vars.simulation_vars[self.current_scenario]['hang_thres']

        i = 1

        sim_start_time = datetime.strptime(datetime.now().strftime('%H:%M:%S.%f')[:-3], '%H:%M:%S.%f')

        while True:

            if self.end or (self.sim_instance != None and self.sim_instance.end): 
                # end of simulation
                self.helper.debug_log("Simulation Time Taken: " + str(datetime.strptime(datetime.now().strftime('%H:%M:%S.%f')[:-3], '%H:%M:%S.%f') - sim_start_time))
                self.helper.debug_log("Killing ArduPilot, Gazebo and QGCS")
                self.kill_all()

                if self.algo == 'drl':
                    self.ac_agent.actor.save_checkpoint(env_vars.ARDU_LOGS + self.current_scenario + '/' + self.session + '/' + self.sim_instance.log_handler.currTime + '/' + self.sim_instance.log_handler.currTime + "_actor")
                    self.ac_agent.critic.save_checkpoint(env_vars.ARDU_LOGS + self.current_scenario + '/' + self.session + '/' +  self.sim_instance.log_handler.currTime + '/' + self.sim_instance.log_handler.currTime + "_critic")

                    # Print Agent's Replay memory
                    self.sim_instance.log_handler.save_memory(self.memory)

                self.sim_instance.log_handler.write_to_file()
                self.helper.debug_log("violations: " + str(self.violation_scores))
                self.helper.debug_log("vio_timestamps: " + str(self.vio_timestamps))
                self.sim_instance.log_handler.save_violation_scores(self.violation_scores, self.vio_timestamps)
                self.sim_instance.log_handler.save_params(self.sim_instance.params_set, self.sim_instance.params_set_timestamps)

                self.plot_results()

                self.reset()
                return

            if not self.started_simulation and self.autopilot_ready_time != None and time.time() - self.autopilot_ready_time > buffer:
                self.helper.debug_log("Creating Simulation Instance")
                if self.ranking:
                    self.sim_instance = Simulation(
                        self.current_scenario, self.session, self.helper, len(self.sim_threads), 
                        list(zip([self.top_k_configs[i]['name'] for i in  range(self.k)], self.initialize_randomized_params())))
                else:
                    self.sim_instance = Simulation(
                        self.current_scenario, self.session, self.helper, len(self.sim_threads), 
                        list(zip([self.top_k_configs[i]['name'] for i in  range(len(env_vars.simulation_vars[self.current_scenario]['params_to_fuzz']))], 
                                self.initialize_randomized_params())))
                self.sim_thread = kthread.KThread(target=self.sim_instance.simulate, daemon=False, name="sim_ins-" + str(len(self.sim_threads)+1))
                self.sim_thread.start()
                self.sim_threads.append(self.sim_thread)
                self.started_simulation = True
                # self.helper.debug_log("Simulation instance created")

            # if the latest record in debug_logs are older than threshold
            last = datetime.strptime(self.helper.last_update, '%H:%M:%S.%f')
            now = datetime.strptime(datetime.now().strftime('%H:%M:%S.%f')[:-3], '%H:%M:%S.%f')
            hang_duration = (now - last).seconds
            if hang_duration > threshold:
                self.helper.debug_log("Hang Threshold Exceeded")
                self.sim_instance.end = True
                self.sim_instance.end_gen = True
                self.end = True
                time.sleep(2)
                # self.hang = True

            print("Simulation #", len(self.sim_threads), "Time :", int(round(time.time()-self.start_time, 0)), ":", hang_duration)
            print('# Threads :', len([t.getName() for t in threading.enumerate()]))
            # print('Threads :', ', '.join(t.getName() for t in threading.enumerate()))

            if self.started_simulation and self.sim_instance.ready:
                # start set job
                if self.sim_instance.ready_to_fuzz and not self.started_setting:
                    self.set_thread = kthread.KThread(target=self.set_job, daemon=False, name="set_ins-" + str(len(self.set_threads)+1))
                    # self.set_thread = kthread.KThread(target=self.set_job_ga, daemon=False, name="set_ins-" + str(len(self.set_threads)+1))
                    self.set_threads.append(self.set_thread)
                    self.set_thread.start()
                    self.started_setting = True

                self.sim_instance.log_handler.log_stream()

                if not self.sim_instance.disarming:
                    # get violation
                    try:
                        # chute1 = round(self.sim_instance.policy_checker.A_CHUTE1(), 3)
                        # rtl1 = round(self.sim_instance.policy_checker.A_RTL1(), 3)
                        # rtl2 = round(self.sim_instance.policy_checker.A_RTL2(), 3)
                        # rtl3 = round(self.sim_instance.policy_checker.A_RTL3(), 3)
                        # rtl4 = round(self.sim_instance.policy_checker.A_RTL4(), 3)
                        # flip1 = round(self.sim_instance.policy_checker.A_FLIP1(), 3)
                        # flip2 = round(self.sim_instance.policy_checker.A_FLIP2(), 3)
                        # flip3 = round(self.sim_instance.policy_checker.A_FLIP3(), 3)
                        # flip4 = round(self.sim_instance.policy_checker.A_FLIP4(), 3)
                        # alt_hold1 = round(self.sim_instance.policy_checker.ALT_HOLD1(), 3)
                        # alt_hold2 = round(self.sim_instance.policy_checker.ALT_HOLD2(), 3)
                        # circle1 = round(self.sim_instance.policy_checker.A_CIRCLE1(), 3)
                        # circle2 = round(self.sim_instance.policy_checker.A_CIRCLE2(), 3)
                        # circle3 = round(self.sim_instance.policy_checker.A_CIRCLE3(), 3)
                        # circle4_6 = round(self.sim_instance.policy_checker.A_CIRCLE4_6(), 3)
                        # circle7 = round(self.sim_instance.policy_checker.A_CIRCLE7(), 3)
                        # land1 = round(self.sim_instance.policy_checker.A_LAND1(), 3)
                        # land2 = round(self.sim_instance.policy_checker.A_LAND2(), 3)
                        # auto1 = round(self.sim_instance.policy_checker.A_AUTO1(), 3)
                        # gps_fs1 = round(self.sim_instance.policy_checker.A_GPS_FS1(), 3)
                        # gps_fs2 = round(self.sim_instance.policy_checker.A_GPS_FS2(), 3)
                        # rc_fs1 = round(self.sim_instance.policy_checker.A_RC_FS1(), 3)
                        # rc_fs2 = round(self.sim_instance.policy_checker.A_RC_FS2(), 3)
                        # sport1 = round(self.sim_instance.policy_checker.A_SPORT1(), 3)
                        # guided1 = round(self.sim_instance.policy_checker.A_GUIDED1(), 3)
                        loiter1 = round(self.sim_instance.policy_checker.A_LOITER1(), 3)
                        # drift1 = round(self.sim_instance.policy_checker.A_DRIFT1(), 3)
                        # brake1 = round(self.sim_instance.policy_checker.A_BRAKE1(), 3)

                        self.vio_timestamps.append(datetime.now().strftime('%H:%M:%S.%f')[:-3])
                        # self.violation_scores['A.CHUTE1'].append(chute1)
                        # self.violation_scores['A.RTL1'].append(rtl1)
                        # self.violation_scores['A.RTL2'].append(rtl2)
                        # self.violation_scores['A.RTL3'].append(rtl3)
                        # self.violation_scores['A.RTL4'].append(rtl4)
                        # self.violation_scores['A.FLIP1'].append(flip1)
                        # self.violation_scores['A.FLIP2'].append(flip2)
                        # self.violation_scores['A.FLIP3'].append(flip3)
                        # self.violation_scores['A.FLIP4'].append(flip4)
                        # self.violation_scores['A.ALT_HOLD1'].append(alt_hold1)
                        # self.violation_scores['A.ALT_HOLD2'].append(alt_hold2)
                        # self.violation_scores['A.CIRCLE1'].append(circle1)
                        # self.violation_scores['A.CIRCLE2'].append(circle2)
                        # self.violation_scores['A.CIRCLE3'].append(circle3)
                        # self.violation_scores['A.CIRCLE4_6'].append(circle4_6)
                        # self.violation_scores['A.CIRCLE7'].append(circle7)
                        # self.violation_scores['A.LAND1'].append(land1)
                        # self.violation_scores['A.LAND2'].append(land2)
                        # self.violation_scores['A.AUTO1'].append(auto1)
                        # self.violation_scores['A.GPS_FS1'].append(gps_fs1)
                        # self.violation_scores['A.GPS_FS2'].append(gps_fs2)
                        # self.violation_scores['A.RC_FS1'].append(rc_fs1)
                        # self.violation_scores['A.RC_FS2'].append(rc_fs2)
                        # self.violation_scores['A.SPORT1'].append(sport1)
                        # self.violation_scores['A.GUIDED1'].append(guided1)
                        self.violation_scores['A.LOITER1'].append(loiter1)
                        # self.violation_scores['A.DRIFT1'].append(drift1)
                        # self.violation_scores['A.BRAKE1'].append(brake1)
                    except Exception as e:
                        # self.helper.debug_log("Policy Checking Exception:" + str(e))
                        print("Policy Checking Exception in main loop:" + str(e))

            # i += 1
            time.sleep(.2)

    def set_job(self):
        self.helper.debug_log("Start Param Fuzzing")
        while True:
            if self.sim_instance == None or self.sim_instance.end:
                break
            try:
                if not self.sim_instance.disarming:
                    if self.algo == 'rand':

                        # send random params
                        if self.ranking:
                            param_tuples = list(
                                zip([self.top_k_configs[i]['name'] for i in range(self.k)], self.initialize_randomized_params()))
                        else:
                            param_tuples = list(
                                zip([self.top_k_configs[i]['name'] 
                                        for i in range(len(env_vars.simulation_vars[self.current_scenario]['params_to_fuzz']))], 
                                        self.initialize_randomized_params()))  
                        self.sim_instance.set_param(param_tuples)
                        time.sleep(env_vars.simulation_vars['set_wait_secs'])
                    
                    else:

                        # get prev n state
                        # state_seq = self.sim_instance.log_handler.get_prev_n_state(env_vars.learn_vars['window_size_lstm'])
                        # get single state instead of seq
                        state = self.sim_instance.log_handler.get_current_state_dict()

                        if state is not None:

                            if self.algo == 'drl':
                                # infer/generate value
                                normed_state = self.normalize_state(state)
                                param_values = self.ac_agent.choose_params(normed_state)
                            elif self.algo == 'ga':
                                # mutate
                                normed_state = self.normalize_state(state)
                                param_values = self.ga_generator.generate(normed_state)['conf'] # will normalize inside

                            rescaled_values = self.rescale_params(param_values.tolist())
                            self.helper.learn_log("Generated Params : " + str(rescaled_values))   

                            # rescale param values from (0, 1) to documentation
                            if self.ranking:
                                param_tuples = list(
                                    zip([self.top_k_configs[i]['name'] for i in range(self.k)], rescaled_values))          
                            else:      
                                param_tuples = list(
                                    zip([self.top_k_configs[i]['name'] 
                                            for i in range(len(env_vars.simulation_vars[self.current_scenario]['params_to_fuzz']))], 
                                            rescaled_values))  
                                                                            
                            self.sim_instance.set_param(param_tuples)

                            time.sleep(env_vars.simulation_vars['set_wait_secs'])
                            if self.sim_instance == None or self.sim_instance.end:
                                break
                            
                            violations = []
                            if self.ranking:
                                violations.append(max(self.violation_scores[self.current_scenario][-env_vars.learn_vars['window_size_vio']:]))
                            else:
                                for key in self.violation_scores.keys():
                                    violations.append(max(self.violation_scores[key][-env_vars.learn_vars['window_size_vio']:]))
                            
                            if self.algo == 'drl':
                                # for cac with lstm
                                new_state_ = self.sim_instance.log_handler.get_prev_n_state_dict(1)
                                    
                                # with replay memory
                                self.memory.push((normed_state, param_values.numpy(),  max(violations), self.normalize_state(new_state_[0])))

                                if len(self.memory) >= self.memory.batch_size:
                                    for i in range(3):
                                        self.ac_agent.learn_batch(self.memory.sample())
                        else:
                            time.sleep(.5)

            except Exception as e:
                self.helper.debug_log("Fuzzing Exception: " + str(e))
                time.sleep(.5)
                
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
        return np.array([_state[i] for i in sorted(state, key=str.lower)])
    
    def normalize_params(self, params):
        _params = []
        i = 0
        for p in params:
            min_val = float(self.top_k_configs[i]['range'][0])
            max_val = float(self.top_k_configs[i]['range'][1])
            _params.append((p - min_val) / (max_val - min_val))
            i += 1
        return _params

    def rescale_params(self, params):
        old_min = 0
        old_max = 1
        _params = []
        i = 0
        if type(params) is not list:
            params = [params]
        for p in params:
            new_min = float(self.top_k_configs[i]['range'][0])
            new_max = float(self.top_k_configs[i]['range'][1])
            rescaled_val = ( ( p - old_min ) / (old_max - old_min) ) * (new_max - new_min) + new_min
            _rescaled_val = float(self.top_k_configs[i]['range'][0])
            if self.top_k_configs[i]['increment'] is not None:
                while _rescaled_val < rescaled_val:
                    _rescaled_val += self.top_k_configs[i]['increment']
                _params.append(round(_rescaled_val, 3))    
            else:
                _params.append(round(_rescaled_val, 3))
            i += 1
        return _params

    def initialize_randomized_params(self):
        params = []
        i = 0
        for p in self.top_k_configs:
            try:
                _r = random.uniform(float(p['range'][0]), float(p['range'][1]))
                if p['increment'] is not None:
                    r = float(p['range'][0])
                    while r < _r:
                        r += p['increment']
                    params.append(round(r, 2))
                else:
                    params.append(_r)
            except Exception as e:
                self.helper.debug_log("Randomization Exception for parameter {}: {}".format(p['name'], str(e)))
        return params

    def wait(self):
        threads = [t for t in threading.enumerate() if t.is_alive()]
        self.helper.debug_log("# Threads : " + str(len(threads)))
        self.helper.debug_log("Threads : " + ', '.join(t.getName() for t in threads))

        prefix_to_check = ['sim_ins-', 'req_param-', 'all_msg-']
        running_sim_threads = [t for t in threading.enumerate() if t.is_alive() and any(x in t.getName() for x in prefix_to_check)]
        # running_sim_threads = [t for t in threading.enumerate() if t.is_alive() and "MainThread" not in str(t.getName())]
        if not len(running_sim_threads) == 0:
            for _t in running_sim_threads:
                self.helper.debug_log("Waiting for " + _t.getName() + " to terminate.")
                _t.join()
            _t = [t for t in threading.enumerate() if t.is_alive()]
            self.helper.debug_log("# Threads : " + str(len(_t)))
            self.helper.debug_log("Threads : " + ', '.join(t.getName() for t in _t))

    def reset(self):
        self.helper.debug_log("Reseting Handler")
        # self.helper = helpers.Helper(self.current_scenario)
        self.start_time = time.time()
        self.qground_process: Popen = None
        self.gazebo_process: Popen = None
        self.ardu_process: Popen = None
        self.sim_thread: kthread.KThread = None
        self.sim_instance = None
        self.autopilot_ready_time = None
        self.started_simulation = False
        self.end = False
        self.violation_scores = deepcopy(env_vars.violation_scores_blank)
        self.vio_timestamps = []
        # self.hang = False
        
        self.set_threads = []
        self.set_thread: kthread.KThread = None
        self.started_setting = False
        
    def plot_results(self):

        try:
            #plot violations
            x_axis = self.vio_timestamps
            
            for vio in self.violation_scores.keys():
                y_axis = self.violation_scores[vio]
                plt.plot(x_axis, y_axis, label=vio)

            plt.title("Policy Violations")
            plt.xlabel('time')
            plt.legend()
            plt.axhline(y=0, color='r', linestyle='--')
            plt.xticks(x_axis[::math.floor(len(x_axis)/10)], rotation="vertical")
            plt.ylim(-1.5, 1.5)
            plt.savefig(self.sim_instance.log_handler.currFolder + self.current_scenario + "_" + self.sim_instance.log_handler.currTime + '.png', bbox_inches='tight')
            plt.clf()

        except Exception as e:
            self.helper.debug_log("Violation Plot Exception: " + str(e))

        try:

            #plot gps trajectory
            lat = [r for r in self.sim_instance.log_handler.streamed_logs['lat']]
            lon = [r for r in self.sim_instance.log_handler.streamed_logs['lon']]

            fig, ax = plt.subplots()
            ax.plot(lon, lat, 'b-')
            ax.set_xlim(min(lon) - .0001, max(lon) + .0001)
            ax.set_ylim(min(lat) - .0001, max(lat) + .0001)

            # #plot violations on trajectory
            # trajectory = list(zip(lon, lat))
            # t = 0
            # vio_pos = 0
            # for ln,lt in trajectory:
            #     while True: 
            #         if vio_pos >= len(self.vio_timestamps):
            #             break
            #         curr_log_time = self.sim_instance.log_handler.merged_logs['time_stamp'][t]
            #         curr_vio_time = self.vio_timestamps[vio_pos][:-4]
            #         if datetime.strptime(curr_vio_time, '%H:%M:%S') >= datetime.strptime(curr_log_time, '%H:%M:%S'):
            #             break
            #         vio_pos += 1
            #     if vio_pos >= len(self.vio_timestamps):
            #         break
            #     if self.violation_scores[self.current_scenario][vio_pos] > 0:
            #         plt.plot(ln, lt, 'or')
            #     t += 1

            plt.title("Trajectory")
            plt.savefig(self.sim_instance.log_handler.currFolder + "trajectory_" + self.sim_instance.log_handler.currTime + '.png')
            plt.clf()

        except Exception as e:
            self.helper.debug_log("Trajectory Plot Exception: " + str(e))

    def kill_all(self):
        self.kill_list(['QGround', 'ardu', 'gz', 'gazebo'])

    def kill_list(self, name_list):
        for name in name_list:
            self.kill_ps(name)

    def kill_ps(self, name):
        output = Popen('ps -ax | grep ' + name, shell=True, stdout=PIPE, stderr=PIPE).communicate()[0]
        matching_processes = output.decode().split('\n')
        # self.helper.debug_log('Matching ps: ' + ' ,'.join(matching_processes))
        for proc in matching_processes:
            if proc != '':
                proc_id = proc.split()[0]
                print('Killing', proc_id)
                Popen('kill -9 ' + proc_id, shell=True)