from datetime import datetime
import datetime as dt
import os
import csv
import helpers
import pandas as pd
import numpy as np
import env_vars as env_vars

class LogHandler():

    def __init__(self, currTime: str, helper, p, curr_scenario, session):
        self.currTime = currTime.replace(":", "_").replace(".", "_")
        self.currFolder = p + curr_scenario + '/' + session + '/' + self.currTime + '/'
        self.current_scenario = curr_scenario
        self.helper: helpers.Helper = helper
        self.opened_logs = {}
        self.merged_logs = {}
        
        self.streamed_logs = {}
        self.streamed_logs['time_stamp'] = []
        for l in env_vars.simulation_vars[self.current_scenario]['log_cols']:
            self.streamed_logs[l] = []
        for l in env_vars.simulation_vars[self.current_scenario]['params_to_fetch']:
            self.streamed_logs[l] = []

        if not os.path.exists(self.currFolder):
            print("Creating", self.currFolder)
            os.makedirs(self.currFolder)
    
    def log(self, name, ts, data):
        if not name in self.opened_logs:
            self.opened_logs[name] = []
        if name == 'lat' or name == 'lon':
            self.opened_logs[name].append((ts, data/10000000.0))
            return
        elif name == 'relative_alt':
            self.opened_logs[name].append((ts, max(0, data)))
            return
        self.opened_logs[name].append((ts, data))

    def get_current_state(self):
        now = datetime.now()
        previous_second = (now - dt.timedelta(0,1)).strftime('%H:%M:%S.%f')[:-7]
        state = {}

        if not self.state_cols_ready():
            return None
        for l in self.opened_logs:
            relevant_records = [r[1] for r in self.opened_logs[l] if previous_second in r[0]]
            if len(relevant_records) == 0:
                return None
            if l in env_vars.simulation_vars[self.current_scenario]['state_cols']:
                state[l] = sum([float(r) for r in relevant_records])/len(relevant_records)
            if l == 'MODE':
                # state[l] = relevant_records[-1]
                # embed one-hot of mode to state 
                for m in env_vars.simulation_vars[self.current_scenario]['modes_vec']:
                    if relevant_records[-1] == m:
                        state["MODE_" + m] = 1
                    else:
                        state['MODE_' + m] = 0
            continue
            
        return [state[i] for i in sorted(state, key=str.lower)]

    def get_prev_state_dict(self, now, t):
        previous_second = (now - dt.timedelta(0,t)).strftime('%H:%M:%S.%f')[:-7]
        state = {}
        
        for l in self.opened_logs:
            relevant_records = [r[1] for r in self.opened_logs[l] if previous_second in r[0]]
            if len(relevant_records) == 0:
                return None
            if l in env_vars.simulation_vars[self.current_scenario]['state_cols']:
                state[l] = sum([float(r) for r in relevant_records])/len(relevant_records)
            if l == 'MODE':
                # state[l] = relevant_records[-1]
                # embed one-hot of mode to state 
                for m in env_vars.simulation_vars[self.current_scenario]['modes_vec']:
                    if relevant_records[-1] == m:
                        state["MODE_" + m] = 1
                    else:
                        state['MODE_' + m] = 0
            continue
        
        return state  

    def get_prev_n_state_dict(self, n):
        if not self.state_cols_ready():
            return None

        now = datetime.now()
        state_seq = []

        try:
            step = 1
            while len(state_seq) < n:
                state = self.get_prev_state_dict(now, step)
                step += 1
                if state == None:
                    continue
                else:
                    state_seq.insert(0, state)
        except Exception as e:
            raise Exception("Previous n state not ready")

        return np.asarray(state_seq)  

    def get_current_state_dict(self):
        now = datetime.now()
        previous_second = (now - dt.timedelta(0,1)).strftime('%H:%M:%S.%f')[:-7]
        state = {}

        if not self.state_cols_ready():
            return None
        for l in self.opened_logs:
            relevant_records = [r[1] for r in self.opened_logs[l] if previous_second in r[0]]
            if len(relevant_records) == 0:
                return None
            if l in env_vars.simulation_vars[self.current_scenario]['state_cols']:
                state[l] = sum([float(r) for r in relevant_records])/len(relevant_records)
            if l == 'MODE':
                # state[l] = relevant_records[-1]
                # embed one-hot of mode to state 
                for m in env_vars.simulation_vars[self.current_scenario]['modes_vec']:
                    if relevant_records[-1] == m:
                        state["MODE_" + m] = 1
                    else:
                        state['MODE_' + m] = 0
            continue
            
        return state

    def get_prev_state(self, now, t):
        previous_second = (now - dt.timedelta(0,t)).strftime('%H:%M:%S.%f')[:-7]
        state = {}
        
        for l in self.opened_logs:
            relevant_records = [r[1] for r in self.opened_logs[l] if previous_second in r[0]]
            if len(relevant_records) == 0:
                return None
            if l in env_vars.simulation_vars[self.current_scenario]['state_cols']:
                state[l] = sum([float(r) for r in relevant_records])/len(relevant_records)
            if l == 'MODE':
                # state[l] = relevant_records[-1]
                # embed one-hot of mode to state 
                for m in env_vars.simulation_vars[self.current_scenario]['modes_vec']:
                    if relevant_records[-1] == m:
                        state["MODE_" + m] = 1
                    else:
                        state['MODE_' + m] = 0
            continue
        
        return [state[i] for i in sorted(state, key=str.lower)]

    def get_prev_n_state(self, n):
        if not self.state_cols_ready():
            return None

        now = datetime.now()
        state_seq = []

        try:
            step = 1
            while len(state_seq) < n:
                state = self.get_prev_state(now, step)
                step += 1
                if state == None:
                    continue
                else:
                    state_seq.insert(0, state)
        except Exception as e:
            raise Exception("Previous n state not ready")

        return np.asarray(state_seq)
    
    def state_cols_ready(self):
        for col in env_vars.simulation_vars[self.current_scenario]['state_cols']:
            if col not in self.opened_logs.keys():
                return False
            else:
                if len(self.opened_logs[col]) == 0:
                    return False
        return True
    
    def log_cols_ready(self):
        for col in env_vars.simulation_vars[self.current_scenario]['params_to_fetch']:
            if col not in self.opened_logs.keys():
                # self.helper.debug_log("msg {}: {}".format(ts, msg))
                print("params_to_fetch: {} Not ready, key error".format(col))
                return False
            else:
                if len(self.opened_logs[col]) == 0:
                    print("params_to_fetch: {} Not ready, value error".format(col))
                    return False
        for col in env_vars.simulation_vars[self.current_scenario]['log_cols']:
            if col not in self.opened_logs.keys():
                print("log_cols: {} Not ready, key error".format(col))
                return False
            else:
                if len(self.opened_logs[col]) == 0:
                    print("log_cols: {} Not ready, value error".format(col))
                    return False
        return True
    
    def log_stream(self):
        try:
            if not self.log_cols_ready():
                raise Exception("Log columns not ready") 
            for k in self.opened_logs.keys():
                self.streamed_logs[k].append(self.opened_logs[k][-1][1])
            self.streamed_logs['time_stamp'].append(datetime.now().strftime('%H:%M:%S.%f')[:-3])
        except Exception as e:
            # self.helper.debug_log("Log stream error: " + str(e))
            print("Log stream error: " + str(e))

    def write_to_file(self):
        try:
            streamDf = pd.DataFrame(self.streamed_logs)
            streamDf.set_index('time_stamp', inplace=True)
            pd.DataFrame(streamDf).to_csv(self.currFolder + 'streamed_logs_' + self.currTime + '.csv')
        except Exception as e:
            self.helper.debug_log("Stream logs write error: " + str(e))

        self.merge()

    def merge(self):
        try:
            self.helper.debug_log("Merging " + str(len(self.opened_logs.keys()))  + " log types")
            intervals = {}
            for log in self.opened_logs:
                for r in self.opened_logs[log]:
                    if r[0][:8] not in intervals:
                        intervals[r[0][:8]] = set([log])
                    else:
                        intervals[r[0][:8]].add(log)
            complete_intervals = [i for i in intervals if len(intervals[i]) == len(self.opened_logs)]
            print('There are ' + str(len(complete_intervals)) + ' complete intervals.')

            self.merged_logs = { 'time_stamp': complete_intervals }
            f_cnt = 0
            for l in self.opened_logs:
                if l not in self.merged_logs:
                    self.merged_logs[l] = []
                f_cnt += 1

                i_cnt = 0

                for ci in complete_intervals:
                    relevant_records = [r[1] for r in self.opened_logs[l] if r[0][:8] == ci]   
                    i_cnt += 1
                    if l == 'MODE':
                        self.merged_logs[l].append([ci, relevant_records[-1]])  
                        continue
                    self.merged_logs[l].append([ci, sum([float(r) for r in relevant_records])/len(relevant_records)])  
                    # print(str(i_cnt) + ' / ' + str(len(complete_intervals)) + " / "  + str(f_cnt) + " / " + str(len(self.opened_logs)) + " : " + ci)
                print("Merging", str(f_cnt) + " / " + str(len(self.opened_logs)))

            with open(self.currFolder + 'merged_logs_' + self.currTime + '.csv', 'w', encoding='UTF8', newline='') as csv_file:
                self.helper.debug_log("Writing " + str(len(self.merged_logs['time_stamp'])) + " records to " + self.currFolder)
                writer = csv.writer(csv_file)
                header = ['time_stamp'] + list(self.opened_logs.keys())
                writer.writerow(header)
                r_cnt = 0
                for i in range(len(self.merged_logs[list(self.merged_logs.keys())[0]])):
                    row = [self.merged_logs['time_stamp'][i]]
                    for f in self.opened_logs:
                        row.append(self.merged_logs[f][i][1])
                    writer.writerow(row)
                    r_cnt += 1 
                return r_cnt
        except:
            raise Exception("Merge Exception") 

    def save_mission(self, mission):
        with open(self.currFolder + 'missions_' + self.currTime + '.csv', 'a', encoding='UTF8', newline='') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(mission)

    def save_violation_scores(self, violation_scores, timestamps):
        try:
            violationsDf = pd.DataFrame(violation_scores, index=timestamps)
            pd.DataFrame(violationsDf).to_csv(self.currFolder + 'violation_scores_' + self.currTime + '.csv')
        except Exception as e:
            self.helper.debug_log("Violation scores record error: " + str(e))

    def save_params(self, params_records, timestamps):
        try:
            paramsDf = pd.DataFrame(params_records, index=timestamps)
            pd.DataFrame(paramsDf).to_csv(self.currFolder + 'params_records_' + self.currTime + '.csv')
        except Exception as e:
            self.helper.debug_log("Params record error: " + str(e))

    def save_memory(self, memory):
        try:
            with open(self.currFolder + 'memory_' + self.currTime, 'wb') as f:
                arr = np.array(memory.memory)
                np.save(f, arr)

        except Exception as e:
            self.helper.debug_log("Memory print error: " + str(e))