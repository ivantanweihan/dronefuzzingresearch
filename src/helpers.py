import os
import pprint
from subprocess import *
import env_vars as env_vars
from datetime import datetime

class Helper():

    def __init__(self, curr_scenario, curr_session = None):
        
        self.last_update = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        self.current_scenario = curr_scenario
        self.current_session = None

    def new_session(self):
        if not os.path.exists(os.path.join(env_vars.ARDU_LOGS,self.current_scenario, self.current_session)):
            print("Creating", os.path.join(env_vars.ARDU_LOGS,self.current_scenario, self.current_session))
            os.makedirs(os.path.join(env_vars.ARDU_LOGS,self.current_scenario, self.current_session))

        self.debug_log("============================================================\n")
        self.debug_log(pprint.pformat(env_vars.simulation_vars[self.current_scenario]))
        self.learn_log("============================================================\n")
        self.learn_log(pprint.pformat(env_vars.learn_vars))

    def new_episode(self, i):
        self.last_update = datetime.now().strftime('%H:%M:%S.%f')[:-3]

        self.debug_log("------------------------------------------------------------")
        self.debug_log("Loop #" + str(i + 1))

        self.learn_log("------------------------------------------------------------")
        self.learn_log("Loop #" + str(i + 1))

    def debug_log(self, s):

        ts = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        self.last_update = ts
        f = open(env_vars.ARDU_LOGS + self.current_scenario + '/' + 
                 self.current_session + "/debug_logs.txt", "a")
        f.write("\n" + ts + " : " + str(s))
        print(str(s))
        f.close()

    def learn_log(self, s):

        ts = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        f = open(env_vars.ARDU_LOGS + self.current_scenario + '/' + 
                 self.current_session + "/learn_logs.txt", "a")
        f.write("\n" + ts + " : " + str(s))
        print(str(s))
        f.close()

    def procedural_generated_log(objs):

        ts = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        f = open(env_vars.PCD_GEN_LOGS + ts, "a")
        for o in objs:
            pass
        f.close()

