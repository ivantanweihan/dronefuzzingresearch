import threading
import time
import random
import env_vars
from policy_checker import PolicyChecker
from helpers import Helper
from pymavlink_log_handler import LogHandler
from datetime import datetime
from pymavlink import mavutil
import mission as mission

global is_disarmed
is_disarmed = False

class Simulation:
        
    def __init__(self, curr_scenario, session, helper, i, params):
    # def __init__(self, curr_scenario, helper, i):

        self.i = i
        self.helper: Helper = helper
        self.log_handler: LogHandler = LogHandler(str(time.time()), self.helper, env_vars.ARDU_LOGS, curr_scenario, session)
        self.current_scenario = curr_scenario

        self.connection :mavutil.mavudp = None

        self.policy_checker = PolicyChecker(self.log_handler)
        
        self.inital_params = params

        self.params_set = { p[0]: [] for p in params}
        self.params_set_timestamps = []

        # TODO: Modify here to add more environment variables claimed in env_vars.py
        self.fields = {
            # alt: MSL altitude mm, relative_alt: relative altitude above ground mm
            'GLOBAL_POSITION_INT': ['alt', 'lat', 'lon', 'relative_alt', 'vx', 'vy', 'vz'],
            'ATTITUDE': ['roll', 'pitch', 'yaw', 'rollspeed', 'pitchspeed', 'yawspeed'],
            # 'VFR_HUD': ['airspeed', 'groundspeed', 'throttle', 'climb', 'heading'],
            # 'VFR_HUD': ['alt'],
            'SCALED_IMU2': ['xacc','yacc','zacc','xgyro','ygyro','zgyro'],
            # 'HIGHRES_IMU': ['xacc','yacc','zacc','xgyro','ygyro','zgyro'],
            'RC_CHANNELS': ['chan1_raw', 'chan2_raw', 'chan3_raw', 'chan4_raw'],
            # 'NAV_CONTROLLER_OUTPUT': ['nav_roll', 'nav_pitch', 'nav_bearing', 'target_bearing', 'wp_dist', 'alt_error'],
            # 'GPS_RAW_INT': ['satellites_visible'],
            'STATUSTEXT': None
        }

        self.status = {
                # 'parachute_on': 0,
                'GPS_status': 1,
                'Gyro_status': 1,
                'Accel_status': 1,
                'Baro_status': 1, 
                'PreArm_error': 0
        }

        self.ready = False
        self.ready_to_fuzz = False
        self.end_mission = False
        self.end_gen = False
        self.end = False

        self.home = (None, None)
        self.disarm_wait = 5
        self.disarm_wait_cnt = 0
        self.disarming = False
        is_disarmed = False
        self.crashed = False
        self.crashsed_wait = 15
        self.crashsed_wait_cnt = 0

        self.mannual_command_cnt = 0

        self.scenario_procedures = {
            # 'A.CHUTE1': {'method':self.test_Chute, 'args':{}},
            'A.RTL1': {'method':self.test_RTL, 'args':{}},
            'A.RTL2': {'method':self.test_RTL, 'args':{}},
            'A.RTL3': {'method':self.test_RTL, 'args':{}},
            'A.RTL4': {'method':self.test_RTL, 'args':{}},
            'A.FLIP1': {'method':self.test_flip, 'args':{}},
            'A.FLIP2': {'method':self.test_flip, 'args':{}},
            'A.FLIP3': {'method':self.test_RTL, 'args':{}},
            'A.FLIP4': {'method':self.test_RTL, 'args':{}},
            'A.ALT_HOLD1': {'method': self.test_Alt_Hold, 'args': {}},
            'A.ALT_HOLD2': {'method': self.test_Alt_Hold, 'args': {}},
            # 'A.CIRCLE1': {'method':self.test_Circle, 'args':{}},
            # 'A.CIRCLE2': {'method':self.test_Circle, 'args':{}},
            # 'A.CIRCLE3': {'method':self.test_Circle, 'args':{}},
            # 'A.CIRCLE4_6': {'method':self.test_Circle, 'args':{}},
            # 'A.CIRCLE7': {'method':self.test_Circle, 'args':{}},
            'A.LAND1': {'method':self.test_land, 'args':{}},
            'A.LAND2': {'method':self.test_RTL, 'args':{}},
            'A.AUTO1': {'method':self.test_auto, 'args':{}},
            # 'A.GPS_FS1': {'method':self.test_GPS_FS, 'args':{}},
            # 'A.GPS_FS2': {'method':self.test_GPS_FS, 'args':{}},
            # 'A.RC_FS1': {'method':self.test_RC_FS, 'args':{}},
            # 'A.RC_FS2': {'method':self.test_RC_FS, 'args':{}},
            'A.SPORT1': {'method':self.test_sport, 'args':{}},
            # 'A.GUIDED1': {'method':self.test_Guided, 'args':{}},
            'A.LOITER1': {'method':self.test_loiter, 'args':{}},
            # 'A.DRIFT1': {'method':self.test_drift, 'args':{}},
            'A.BRAKE1': {'method':self.test_brake, 'args':{}},
        }

        self.objectives = [] # home is the first and second item, first objective is the 3rd item, and so forth

    def init_connection(self):
        try:
            self.connection = mavutil.mavlink_connection('udpin:localhost:14551', udp_timeout=15)
            self.connection.wait_heartbeat(timeout=15)
        except TimeoutError:
            raise TimeoutError("MAVLink connection timeout")
        print("Heartbeat from system (system %u component %u)" % (self.connection.target_system, self.connection.target_component))
    
    def set_param(self, param_tuples):
        self.params_set_timestamps.append(datetime.now().strftime('%H:%M:%S.%f')[:-3])

        for pt in param_tuples:
            try:
                self.connection.param_set_send(pt[0], pt[1])
                self.params_set[pt[0]].append(pt[1])
                # self.helper.debug_log("Setting Param: " + str(pt[0]) + " at " + str(pt[1]))
            except Exception as e:
                self.helper.debug_log("Param setting exception : " + str(e))

    def simulate(self):
        try:

            self.init_connection()

            # Wait for IMU in the STATUSTEXT
            if self.ready_to_fly():
                
                # for param in env_vars.simulation_vars[self.current_scenario]['constant_params'].keys():
                #     self.connection.param_set_send(param, env_vars.simulation_vars[self.current_scenario]['constant_params'][param])

                self.ready = True

                self.start_logging()

                if self.end or self.end_gen:
                    return

                procedure = self.scenario_procedures[self.current_scenario]['method']
                arguments = self.scenario_procedures[self.current_scenario]['args']
                procedure(**arguments)
                self.helper.debug_log(f"Calling procedure {procedure} with arguments: {arguments}")

        except TimeoutError as te:
            self.helper.debug_log("Timeout Error:" + str(te))
        except Exception as e:
            self.helper.debug_log("Simulation Error:" + str(e))
        finally:
            self.connection.param_set_send('FORMAT_VERSION', 0)
            self.connection.waypoint_clear_all_send()
            self.end_gen = True
            self.helper.debug_log("Ending Simulation Finally")
            time.sleep(2)
            self.end = True

    def test_RTL(self):
        # RTL Scenarios
        try:

            self.set_mission()
            self.start_mission()
            
            # self.ready_to_fuzz = True

            # enable EK2
            self.connection.param_set_send('EK2_ENABLE', 1)
            self.reboot()
            self.init_connection()

            self.ready_to_fuzz = True 

            while not self.end_mission:
                time.sleep(1)
                if self.disarming or self.end or self.end_gen:
                    break

            if not self.disarming:
                self.set_mode('RTL')
                self.helper.debug_log("Current RTL_ALT : " + str(self.log_handler.opened_logs['RTL_ALT'][-1][1]))

            while True:
                time.sleep(1)
                self.helper.debug_log("Current mode is " + str(self.log_handler.opened_logs['MODE'][-1][1]))
                if self.disarming:
                    self.disarm_wait_cnt += 1
                    if self.disarm_wait_cnt > self.disarm_wait:
                        self.helper.debug_log("Disarm wait over")
                        self.end_gen = True
                        break
                if self.end:
                    self.end_gen = True
                    time.sleep(2)
                    self.helper.debug_log("Ending Test RTL Procedure")
                    break

        except Exception as e:
            raise Exception("RTL exception : " + str(e))

        finally:
            self.end_gen = True
            self.end = True

    def test_Alt_Hold(self):
        # First use the RTL scenario
        try:

            self.set_mission()
            self.start_mission()
            # enable EK2
            self.connection.param_set_send('EK2_ENABLE', 1)
            self.reboot()
            self.init_connection()

            self.helper.debug_log("Autopilot rebooted and ready.")

            while not self.end_mission:
                time.sleep(1)
                if self.disarming or self.end or self.end_gen:
                    break

            if not self.disarming:
                self.set_mode('RTL')
                self.helper.debug_log("Current RTL_ALT : " + str(self.log_handler.opened_logs['RTL_ALT'][-1][1]))
            
            self.ready_to_fuzz = True

            while True:
                time.sleep(1)
                if self.disarming:
                    self.disarm_wait_cnt += 1
                    if self.disarm_wait_cnt > self.disarm_wait:
                        self.helper.debug_log("Disarm wait over")
                        self.end_gen = True
                        break
                if self.end:
                    self.end_gen = True
                    time.sleep(2)
                    self.helper.debug_log("Ending Test RTL Procedure")
                    break

        except Exception as e:
            raise Exception("RTL exception : " + str(e))

        finally:
            self.end_gen = True
            self.end = True

    def test_flip(self):
        def send_flip_command():
            flip_channel = 7
            base_mode = random.choice(['STABILIZE', 'ALTHOLD'])
            self.set_mode(base_mode)
            self.helper.debug_log(f"Mode set to {base_mode} for Flip operations.")

            time.sleep(3)


            # Activate Flip Mode by setting the RC channel to ON
            self.helper.debug_log("Activating Flip Mode.")
            self.set_rc_channel_pwm(flip_channel, 2000)

            self.set_rc_channel_pwm(1, 2300)
            self.set_mode("FLIP")

            # Hold the flip command for a short duration to trigger the flip
            flip_duration = 3  # seconds; adjust as needed
            time.sleep(flip_duration)

            # Reset the RC channel to OFF to prevent continuous flipping
            self.helper.debug_log("Deactivating Flip Mode.")
            self.set_rc_channel_pwm(flip_channel, 1000)

            # Set the mode back to AUTO to continue the mission
            self.set_mode("AUTO")

        try:
            self.helper.debug_log("Starting Flip Test Scenario.")

            # Initialize the mission using the existing set_mission method
            self.set_mission()
            self.start_mission()

            # Enable EKF2 and Flip capabilities
            self.connection.param_set_send('EK2_ENABLE', 1)
            self.connection.param_set_send('FLIP_ENABLE', 1)
            self.helper.debug_log("EKF2 and FLIP capabilities enabled.")

            # Configure RC Channel for Flip Mode
            flip_channel = 7  # Assuming Channel 7 is used for Flip; adjust if necessary
            self.connection.param_set_send(f'RC{flip_channel}_OPTION', 2)  # 2 = Flip
            self.helper.debug_log(f"RC Channel {flip_channel} configured for Flip Mode.")

            # Reboot to apply parameter changes
            self.reboot()
            self.init_connection()
            
            time.sleep(30)  # Wait for the mission to start

            self.ready_to_fuzz = True 
            while not self.end_mission:
                time.sleep(random.randint(2, 10))  # Random delay between flips
                if self.disarming or self.end or self.end_gen:
                    break
                send_flip_command()

            if not self.disarming:
                self.set_mode('LAND')
                self.helper.debug_log("Mode set to LAND")
            
            while True:
                time.sleep(1)
                if self.disarming:
                    self.disarm_wait_cnt += 1
                    if self.disarm_wait_cnt > self.disarm_wait:
                        self.helper.debug_log("Disarm wait over")
                        self.end_gen = True
                        break
                if self.end:
                    self.end_gen = True
                    time.sleep(2)
                    self.helper.debug_log("Ending Flip Test Scenario.")
                    break
            
        except Exception as e:
            raise Exception("Flip exception : " + str(e))
        
        finally:
            self.end_gen = True
            self.end = True

    def test_land(self):
        try:
            self.connection.waypoint_clear_all_send()

            self.helper.debug_log("Autopilot ready.")

            # set mode to guided
            self.set_mode('GUIDED')
            time.sleep(3)

            TAKE_OFF_ALT = 100
            self.arm()
            # ack_msg = self.connection.recv_match(type='COMMAND_ACK', blocking=True)
            self.helper.debug_log("Arming complete")

            time.sleep(3)
            self.takeoff(TAKE_OFF_ALT)
            # ack_msg = self.connection.recv_match(type='COMMAND_ACK', blocking=True)
            # while ack_msg is None or ack_msg.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            #     self.helper.debug_log("Takeoff command not accepted, retrying")
            #     self.takeoff(TAKE_OFF_ALT)
            #     ack_msg = self.connection.recv_match(type='COMMAND_ACK', blocking=True)
            self.helper.debug_log("Takeoff command issued")

            wait_time = 30
            # self.arm_takeoff(TAKE_OFF_ALT, wait_time)
            time.sleep(wait_time)
            # self.helper.debug_log("Takeoff complete")
            
            self.connection.param_set_send('WPNAV_SPEED_DN', 90)
            time.sleep(3)
            self.connection.param_set_send('LAND_SPEED_HIGH', 90)
            self.helper.debug_log("Landing speed set to 90 cm/s")
            time.sleep(3)

            # set mode to land
            self.set_mode('LAND')
            # ack_msg = self.connection.recv_match(type='COMMAND_ACK', blocking=True)
            self.helper.debug_log("Mode set to LAND")

            self.ready_to_fuzz = True

            # issue land command
            self.land()
            self.helper.debug_log("Land command issued")

            while True:
                time.sleep(1)
                if self.disarming:
                    self.disarm_wait_cnt += 1
                    if self.disarm_wait_cnt > self.disarm_wait:
                        self.helper.debug_log("Disarm wait over")
                        self.end_gen = True
                        break
                if self.end:
                    self.end_gen = True
                    time.sleep(2)
                    self.helper.debug_log("Ending Test Land Procedure")
                    break

        except Exception as e:
            raise Exception("LAND exception : " + str(e))

        finally:
            self.end_gen = True
            self.end = True

    def test_auto(self):
        def send_yaw_command():
            '''
            Choose a command for roll, pitch and throttle
            '''
            yaw = random.randint(-1000, 1000)
            self.manual(500, 500, 500, yaw)
            self.helper.debug_log("Yaw command sent: " + str(yaw))

            duration = random.randint(1, 3)
            time.sleep(duration)  # maintain the yaw for a random duration

            # Reset the RC channel
            self.connection.set_mode_auto()
            time.sleep(10)

        try:
            self.helper.debug_log("Starting AUTO Test Scenario.")

            # Initialize the mission using the existing set_mission method
            self.set_mission()
            self.start_mission()
            
            time.sleep(10)  # Wait for the mission to start

            self.ready_to_fuzz = True 
            while not self.end_mission:
                # time.sleep(random.randint(4, 10))  # Random delay between command executions
                if self.disarming or self.end or self.end_gen:
                    break
                send_yaw_command()
            
            self.set_mode('LAND')
            self.helper.debug_log("Ending AUTO Test Scenario.")
            return
            
            # while True:
            #     time.sleep(1)
            #     if self.disarming:
            #         self.disarm_wait_cnt += 1
            #         if self.disarm_wait_cnt > self.disarm_wait:
            #             self.helper.debug_log("Disarm wait over")
            #             self.end_gen = True
            #             break
            #     if self.end:
            #         self.end_gen = True
            #         time.sleep(2)
            #         self.helper.debug_log("Ending AUTO Test Scenario.")
            #         break
            
        except Exception as e:
            raise Exception("AUTO exception : " + str(e))
        
        finally:
            self.end_gen = True
            self.end = True

    def test_brake(self):
        pass

    def test_loiter(self):
        def send_loiter_command():
            self.set_mode('LOITER')
            self.helper.debug_log("Mode set to LOITER")

            time.sleep(5)  # Loiter for 5 seconds

            self.helper.debug_log("Deactivating Loiter Mode.")
            self.set_mode('AUTO')

        try: 
            self.helper.debug_log("Starting Loiter Test Scenario.")
            self.set_mission()
            self.start_mission()
            time.sleep(20)  # Wait for the mission to start
            self.ready_to_fuzz = True
            while not self.end_mission:
                time.sleep(random.randint(2, 10))  # Random delay loiter commands
                if self.disarming or self.end or self.end_gen:
                    break
                send_loiter_command()

            if not self.disarming:
                self.set_mode('LAND')
                self.helper.debug_log("Mode set to LAND")
            
            while True:
                time.sleep(1)
                if self.disarming:
                    self.disarm_wait_cnt += 1
                    if self.disarm_wait_cnt > self.disarm_wait:
                        self.helper.debug_log("Disarm wait over")
                        self.end_gen = True
                        break
                if self.end:
                    self.end_gen = True
                    time.sleep(2)
                    self.helper.debug_log("Ending Loiter Test Scenario.")
                    break
            
        except Exception as e:
            raise Exception("Loiter exception : " + str(e))
        
        finally:
            self.end_gen = True
            self.end = True

    def test_sport(self):
        try:
            self.connection.waypoint_clear_all_send()

            self.helper.debug_log("Autopilot ready.")

            # set mode to guided
            self.set_mode('GUIDED')
            time.sleep(3)

            TAKE_OFF_ALT = 100
            self.arm()
            # ack_msg = self.connection.recv_match(type='COMMAND_ACK', blocking=True)
            self.helper.debug_log("Arming complete")

            time.sleep(3)
            self.takeoff(TAKE_OFF_ALT)
            # ack_msg = self.connection.recv_match(type='COMMAND_ACK', blocking=True)
            # while ack_msg is None or ack_msg.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            #     self.helper.debug_log("Takeoff command not accepted, retrying")
            #     self.takeoff(TAKE_OFF_ALT)
            #     ack_msg = self.connection.recv_match(type='COMMAND_ACK', blocking=True)
            self.helper.debug_log("Takeoff command issued")

            wait_time = 20
            # self.arm_takeoff(TAKE_OFF_ALT, wait_time)
            time.sleep(wait_time)
            self.helper.debug_log("Takeoff complete")

            self.ready_to_fuzz = True
            
            # test sport mode for 60 senconds
            self.set_mode('SPORT')
            self.helper.debug_log("Mode set to SPORT")
            
            # use max throttle for 120 seconds
            self.manual(0, 0, 1000, 0)
            time.sleep(120)

            # set mode to land
            # self.set_mode('LAND')
            # # ack_msg = self.connection.recv_match(type='COMMAND_ACK', blocking=True)
            # self.helper.debug_log("Mode set to LAND")

            # # issue land command
            # self.land()
            # self.helper.debug_log("Land command issued")

            # while True:
            #     time.sleep(1)
            #     if self.disarming:
            #         self.disarm_wait_cnt += 1
            #         if self.disarm_wait_cnt > self.disarm_wait:
            #             self.helper.debug_log("Disarm wait over")
            #             self.end_gen = True
            #             break
            #     if self.end:
            #         self.end_gen = True
            #         time.sleep(2)
            #         self.helper.debug_log("Ending Test Land Procedure")
            #         break

        except Exception as e:
            raise Exception("SPORT exception : " + str(e))

        finally:
            self.end_gen = True
            self.end = True

    def test_brake(self):
        def send_brake_command():
            self.set_mode('BRAKE')
            self.helper.debug_log("Mode set to BRAKE")

            time_to_brake = 4
            time.sleep(time_to_brake)  # Give it x seconds to brake

            # Give the time for policy checker to check if the drone is already stopped
            self.set_mode('ALT_HOLD')
            time.sleep(2)

            self.helper.debug_log("Deactivating Brake Mode.")
            self.set_mode('AUTO')

        try:
            self.helper.debug_log("Starting BRAKE Test Scenario.")

            # Initialize the mission using the existing set_mission method
            self.set_mission()
            self.start_mission()
            
            time.sleep(10)  # Wait for the mission to start

            self.ready_to_fuzz = True 
            while not self.end_mission:
                time.sleep(random.randint(5, 20))  # Random delay between command executions
                if self.disarming or self.end or self.end_gen:
                    break
                send_brake_command()
            
            self.set_mode('LAND')
            self.helper.debug_log("Ending BRAKE Test Scenario.")
            return

        except Exception as e:
            raise Exception("BRAKE exception : " + str(e))

        finally:
            self.end_gen = True
            self.end = True

    def test_chute(self):
        pass

    def set_mission(self):
                
        self.connection.waypoint_clear_all_send()

        seq_list = []
        mission_items = []

        mission_items = mission.generate(
            self.current_scenario,
            self.connection.target_system, 
            self.connection.target_component
        )
        self.connection.waypoint_count_send(len(mission_items))
        seq_list = [True] * len(mission_items)
        self.sim_mission_wps = len(mission_items)

        try:
            while True in seq_list:
                msg = self.connection.recv_match(type='MISSION_REQUEST', blocking=True, timeout=15)
                if msg is not None and seq_list[msg.seq] is True:
                    self.log_handler.save_mission([
                        msg.seq, 
                        mission_items[msg.seq].x,
                        mission_items[msg.seq].y,
                        mission_items[msg.seq].z
                        ])
                    self.helper.debug_log('Sending waypoint: ' + str(msg.seq) + 
                    ' | lat : ' + str(mission_items[msg.seq].x) + 
                    ' | lon : ' + str(mission_items[msg.seq].y) + 
                    ' | alt : ' + str(mission_items[msg.seq].z))
                    self.connection.mav.send(mission_items[msg.seq])
                    seq_list[msg.seq] = False
                else:
                    raise TimeoutError("Mission request timeout")
            mission_ack_msg = self.connection.recv_match(type='MISSION_REQUEST', blocking=True, timeout=15)
            self.helper.debug_log("Finished uploading missions")
        except TimeoutError:
            raise TimeoutError("Mission Request/Upload Timeout")

    def start_mission(self):
        self.helper.debug_log('Starting Mission')
        self.arm()
        self.connection.set_mode_auto()

    def ready_to_fly(self):
        while True:
            msg = self.connection.recv_match(type='STATUSTEXT', blocking=True, timeout=30)
            # Retry 5 times
            count = 0
            while (msg == None and count < 5):
                count += 1
                msg = self.connection.recv_match(type='STATUSTEXT', blocking=True, timeout=30)
                self.log_handler.helper.debug_log("msg received from STATUSTEXT is none, retrying count {}".format(count))

            try:
                msg = msg.to_dict()['text']
            except Exception as e:
                self.log_handler.helper.debug_log("Error processing STATUSTEXT, msg received is None")
                raise TimeoutError("Error receiving message from STATUSTEXT, aborting")
                # self.log_handler.helper.debug_log("msg in ready_to_fly is None")
            if "IMU0 is using GPS" in msg:
                self.helper.debug_log("Ready to Fly: IMU using GPS")
                return True

    def arm(self):
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0)

    def disarm(self):
        print('Sending disarm command')
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, 0, 0, 0, 0, 0, 0)

    def takeoff(self, altitude):
        self.helper.debug_log('Taking Off')

        self.connection.mav.command_long_send(
            self.connection.target_system,  # target_system
            self.connection.target_component,  # target_component
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,  # command
            0,  # confirmation
            0,  # param1
            0,  # param2
            0,  # param3
            0,  # param4
            0,  # param5
            0,  # param6
            altitude)  # param7- altitude

    def reboot(self):
        self.helper.debug_log('Rebooting')
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            0,  # Confirmation
            1,  # Reboot autopilot (1)
            0, 0, 0, 0, 0, 0  # Unused parameters
        )


    def land(self):
        print('Sending land command')

        self.connection.mav.command_long_send(
            self.connection.target_system,  # target_system
            self.connection.target_component,  # target_component
            mavutil.mavlink.MAV_CMD_NAV_LAND,  # command
            0,  # confirmation
            0,  # param1
            0,  # param2
            0,  # param3
            0,  # param4
            0,  # param5
            0,  # param6
            0)  # param7- altitude
            
    def set_mode(self, mode):
        try:
            self.helper.debug_log('Setting mode to ' + mode)
            mode_id = self.connection.mode_mapping()[mode]
            self.helper.debug_log('Mode id is ' + mode)

            self.connection.mav.set_mode_send(
                self.connection.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id)
        except:
            self.helper.debug_log('Mode change exception')

    def manual(self, x,y,z,r):
        # https://www.ardusub.com/developers/pymavlink.html#send-rc-joystick
        '''
        # Warning: Because of some legacy workaround, z will work between [0-1000]
        # where 0 is full reverse, 500 is no output and 1000 is full throttle.
        # x,y and r will be between [-1000 and 1000].
        x: Roll control (lateral movement).
        y: Pitch control (forward/backward movement).
        z: Throttle control (altitude or speed control).
        r: Yaw control (rotation around the vertical axis).
        '''
        try:
            self.connection.mav.manual_control_send(self.connection.target_system,x,y,z,r, 0)
        except:
            print("Error with manual input!")

    def arm_takeoff(self, altitude, b2):
        self.helper.debug_log("Arming")
        self.arm() 
        time.sleep(1)
        self.helper.debug_log("Taking off")
        self.takeoff(altitude)
        time.sleep(b2)
        self.helper.debug_log("Take off complete")

    def set_rc_channel_pwm(self, id, pwm=1500):
        """ Set RC channel pwm value
        Args:
            id (TYPE): Channel ID
            pwm (int, optional): Channel pwm value 1100-1900
        """

        try:
            if id < 1:
                print("Channel does not exist.")
                return

            # We only have 8 channels
            # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
            if id < 9:
                rc_channel_values = [65535 for _ in range(8)]
                rc_channel_values[id - 1] = pwm

                #global self.connection

                self.connection.mav.rc_channels_override_send(
                    self.connection.target_system,            # target_system
                    self.connection.target_component,         # target_component
                    *rc_channel_values)            # RC channel list, in microseconds.
        except:
            self.helper.debug_log('RC channel exception')

    def fuzz_rc(self, throttle_exclude=True):

        # https://www.ardusub.com/operators-manual/rc-input-and-output.html#rc-inputs

        # random_throttle = random.randint(1200, 1800)
        # print("Channel", 3, ":", random_throttle)
        # self.set_rc_channel_pwm(3, random_throttle)
        targets = [1, 2, 3, 4]

        if throttle_exclude:
            targets = [1, 2, 4]
            self.set_rc_channel_pwm(3, 1500)

        rpy_target = random.choice(targets)
        random_rpy = random.randint(1200, 1800)
        print("Channel", rpy_target, ":", random_rpy)
        self.set_rc_channel_pwm(rpy_target, random_rpy)

    def throttle_th(self):
        while True:
            if self.end:
                break
            self.set_rc_channel_pwm(3, 1500)
            time.sleep(.2)

    def start_logging(self): 
        self.helper.debug_log("Start State Logging")
        req_params_thread = threading.Thread(target=self.req_params, name='req_param-'+str(self.i+1))
        req_params_thread.start()
        thread = threading.Thread(target=self.get_any_msg, name='all_msg-'+str(self.i+1))
        thread.start()

    def get_any_msg(self):
        while True:
            if self.end_gen or self.end:
                break

            try:
                msg = self.connection.recv_match(blocking=True, timeout=5)

                if msg is not None:
                    # self.helper.debug_log("msg: " + str(msg))
                    
                    ts = datetime.now().strftime('%H:%M:%S.%f')[:-3]

                    # self.helper.debug_log("msg {}: {}".format(ts, msg))
                    

                    # log params for debugging purposes
                    if msg.get_type() == 'PARAM_VALUE':
                        param = msg.to_dict()['param_id']
                        if param in env_vars.simulation_vars[self.current_scenario]['params_to_fetch']: 
                            self.log_handler.log(param, ts, msg.to_dict()['param_value'])

                    if msg.get_type() != "BAD_DATA" and (msg.get_type() in self.fields.keys()):
                        if msg.get_type() == "STATUSTEXT":
                            # if 'Parachute: Released' in msg.text:
                                # self.status['parachute_on'] = 1
                            if 'NavEKF' in msg.text and "Lane switch" in msg.text:
                                self.helper.debug_log(msg.text)
                                self.status['GPS_status'] = 0
                            if 'Vibration Compensation ON' in msg.text:
                                self.helper.debug_log(msg.text)
                                self.status['Gyro_status'] = 0
                            if 'EKF primary changed' in msg.text:
                                self.helper.debug_log(msg.text)
                                self.status['Accel_status'] = 0
                            if 'PreArm: Waiting for Nav Checks' in msg.text:
                                self.helper.debug_log(msg.text)
                                self.status['Baro_status'] = 0
                            if 'PreArm: Check' in msg.text:
                                self.helper.debug_log(msg.text)
                                self.status['PreArm_error'] = 1
                            if 'Crash:' in msg.text:
                                self.helper.debug_log(msg.text)
                                self.crashed = True
                            if 'warning' in msg.text.lower() or 'thrust loss' in msg.text.lower() or 'emergency' in msg.text.lower():
                                self.helper.debug_log(msg.text)
                            # if 'Mission: ' in msg.text:#Reached command
                            if 'reached command' in msg.text.lower():
                                self.helper.debug_log(msg.text)
                                if int(msg.text.split('#')[1]) >= self.sim_mission_wps-1 + self.mannual_command_cnt:
                                    self.end_mission = True
                                    self.helper.debug_log("End of mission simulations")
                            if 'disarm' in msg.text.lower():
                                self.helper.debug_log(msg.text)
                                self.disarming = True
                                is_disarmed = True
                                self.ready_to_fuzz = False
                        else:
                            for attr in self.fields[msg.get_type()]:
                                # Special treatment for barometer height reading as it conflicts with the gps height reading
                                if msg.get_type() == "VFR_HUD" and attr == "alt":
                                    self.log_handler.log("VFR_HUD_alt", ts, getattr(msg, attr))
                                    self.log_handler.debug_log("VFR_HUD: ", getattr(msg, attr))
                                    continue
                                if attr in env_vars.simulation_vars[self.current_scenario]['log_cols']:
                                    self.log_handler.log(attr, ts, getattr(msg, attr))

                    # if msg.get_type() == 'VFR_HUD':
                    #     self.log_handler.log('parachute_on', ts, self.status['parachute_on'])
                    #     self.log_handler.log('GPS_status', ts, self.status['GPS_status'])
                    #     self.log_handler.log('Gyro_status', ts, self.status['Gyro_status'])
                    #     self.log_handler.log('Accel_status', ts, self.status['Accel_status'])
                    #     self.log_handler.log('Baro_status', ts, self.status['Baro_status'])
                    #     self.log_handler.log('PreArm_error', ts, self.status['PreArm_error'])
                    
                    if msg.get_type() == 'HEARTBEAT':
                        self.log_handler.log('MODE', ts, mavutil.mode_string_v10(msg))
            except:
                continue

    def req_params(self):
        while True:
            if self.end or self.end_gen:
                break
            try:
                self.connection.param_fetch_all()
                time.sleep(1)
            except:
                self.helper.debug_log("Req Param Exception")
            