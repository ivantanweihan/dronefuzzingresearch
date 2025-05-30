from pymavlink_log_handler import LogHandler
import geopy.distance as geo_dist
from datetime import datetime
import math

class PolicyChecker():

    def __init__(self, logHandler) -> None:
        self.logHandler: LogHandler = logHandler

    # check if different horizontal position
    @staticmethod
    def relative_displacement(pos1, pos2, threshold=2):
        dist = geo_dist.distance(pos1, pos2).ft
        return (dist-threshold)/threshold

    # check if same horizontal position
    @staticmethod
    def relative_proximity(pos1, pos2, threshold=2):
        dist = geo_dist.distance(pos1, pos2).ft
        return max((threshold-dist)/threshold, -1)

    # for altitude
    @staticmethod
    def alt_change(old_alt: int, current_alt: int, threshold=500) -> int: #threshold in mm
        delta = abs(current_alt-old_alt)
        return (delta-threshold)/threshold
    
    @staticmethod
    def is_descending(old_alt, current_alt, threshold=500): #threshold in mm
        delta = (current_alt-old_alt)
        if delta <= 0: # if indeed descending
            return delta/threshold
        else:
            return delta/threshold
    
    @staticmethod
    def is_ascending(old_alt, current_alt, threshold=500): #threshold in mm
        delta = (current_alt-old_alt)
        if delta >= 0: # if indeed ascending
            return delta/threshold
        else:
            return delta/threshold
    
    @staticmethod
    def not_descending(old_alt, current_alt, threshold=500): #threshold in mm
        delta = (current_alt-old_alt)
        if delta >= 0: # not descending
            return delta/threshold
        else:
            return delta/threshold
    
    @staticmethod
    def not_ascending(old_alt, current_alt, threshold): #threshold in mm
        delta = (current_alt-old_alt)
        if delta > 0: # ascending
            return delta/threshold
        else:
            return delta/threshold
        
    def get_prev(self, log, seconds=1):
        current_time = self.logHandler.opened_logs[log][-1][0]
        for i in range(len(self.logHandler.opened_logs[log])-1, -1, -1):
            t = self.logHandler.opened_logs[log][i][0]
            if (datetime.strptime(current_time, '%H:%M:%S.%f') - datetime.strptime(t, '%H:%M:%S.%f')).seconds >= seconds:
                return self.logHandler.opened_logs[log][i][1]
        raise IndexError("Prev Log Values not ready yet")
    
    def get_prev_flight_mode(self, current_mode):
        '''
        Get the previous flight mode before the current flight mode
        '''
        log_length = len(self.logHandler.opened_logs['MODE'])
        for i in range(log_length-2, 1, -1):
            if self.logHandler.opened_logs['MODE'][i][1] == current_mode:
                continue
            else:
                return self.logHandler.opened_logs['MODE'][i][1]
        raise IndexError("Prev Log Values not ready yet")
    
    def get_yaw_before_flip(self):
        '''
        Get the yaw value before the flip
        '''
        log_length = len(self.logHandler.opened_logs['yaw'])
        for i in range(log_length-2, 1, -1):
            if self.logHandler.opened_logs['MODE'][i][1] == "FLIP":
                continue
            else:
                return self.logHandler.opened_logs['yaw'][i][1]
        raise IndexError("Prev Log Values not ready yet")

    def get_roll_before_flip(self):
        '''
        Get the roll value before the flip
        '''
        log_length = len(self.logHandler.opened_logs['roll'])
        for i in range(log_length-2, 1, -1):
            if self.logHandler.opened_logs['MODE'][i][1] == "FLIP":
                continue
            else:
                return self.logHandler.opened_logs['roll'][i][1]
        raise IndexError("Prev Log Values not ready yet")
    
    def get_pitch_before_flip(self):
        '''
        Get the pitch value before the flip
        '''
        log_length = len(self.logHandler.opened_logs['pitch'])
        for i in range(log_length-2, 1, -1):
            if self.logHandler.opened_logs['MODE'][i][1] == "FLIP":
                continue
            else:
                return self.logHandler.opened_logs['pitch'][i][1]
        raise IndexError("Prev Log Values not ready yet")
    
    def get_flip_start_time(self):
        '''
        Get the time the flip started
        '''
        log_length = len(self.logHandler.opened_logs['MODE'])
        flip_flag = False
        for i in range(log_length-2, 1, -1):
            if self.logHandler.opened_logs['MODE'][i][1] == "FLIP":
                flip_flag = True
                continue
            else:
                if flip_flag:
                    return self.logHandler.opened_logs['MODE'][i][0]
                else:
                    continue
        raise IndexError("Prev Log Values not ready yet")


    def A_CHUTE1(self):
        # Ignore chute first as it has preconditions
        pass

    def A_RTL1(self):
        self.logHandler.helper.debug_log("A_RTL1 executed")
        if not self.logHandler.log_cols_ready():
            raise KeyError("Log Values not ready yet")
        self.logHandler.helper.debug_log("A_RTL1: Log values check passed")
        violation = -1

        # {(ALTt <RTL_ALT)∧(Modet =RTL)→(ALTt−1 <ALTt)}
        # If the current altitude is less than RTL_ALT, then the altitude must be increased until
        # the altitude is greater or equal to the RTL_ALT.
        try:
            P = {}

            # P0: current location != home location
            home_lat = self.logHandler.opened_logs['lat'][0][1]
            home_lon = self.logHandler.opened_logs['lon'][0][1]
            curr_lat = self.logHandler.opened_logs['lat'][-1][1]
            curr_lon = self.logHandler.opened_logs['lon'][-1][1]
            # P[0] = PolicyChecker.relative_displacement((home_lat, home_lon), (curr_lat, curr_lon), 1) 
            
            # PG_FUZZ implementation of P[0] (not exactly the same)
            P[0] = PolicyChecker.relative_displacement((home_lat, home_lon), (curr_lat, curr_lon), 1) 
            if P[0] <= 0:
                P[0] = -1
            else:
                P[0] = 1

            # P1: if current alt < RTL_ALT
            # RTL_ALT is recorded as centimeters
            rtl_alt = self.logHandler.opened_logs['RTL_ALT'][-1][1] * 10 // 1000  #convert to meters
            # relative_alt is recorded as millimeters
            curr_alt = self.logHandler.opened_logs['relative_alt'][-1][1] // 1000  # convert to meter
            # if rtl_alt > 0:
            #     P[1] = 1 if curr_alt < rtl_alt else -1
            # else:
            #     P[1] = -1
            if rtl_alt > 0:
                P[1] = (rtl_alt - curr_alt) / rtl_alt
            else:
                P[1] = 0

            # PG_FUZZ implementation

            # P2: mode == RTL
            if self.logHandler.opened_logs['MODE'][-1][1] == "RTL":
                P[2] = 1
            else:
                P[2] = -1

            self.logHandler.helper.debug_log("RLT_ALT:{}, Relative_alt:{}".format(rtl_alt, curr_alt))
            
            #P3: current alt <= previous alt (negated)
            previous_alt = self.get_prev('relative_alt') // 1000  # convert to meters
            current_alt = self.logHandler.opened_logs['relative_alt'][-1][1] // 1000  # convert to meters
            # P[3] = PolicyChecker.is_descending(previous_alt, current_alt, -100)
            # P[3] = (previous_alt-current_alt)/previous_alt
            # P[3] = 1 if previous_alt >= current_alt else -1

            # PG FUZZ implementation
            if previous_alt != 0:
                P[3] = (previous_alt - current_alt) / previous_alt
            else:
                P[3] = 0

            violation = max(min(P[0], P[1], P[2], P[3]), -1)
            # violation = min(P[0], P[1], P[2], P[3])

            # if violation > 0:
            # print('RTL1:', P[0], P[1], P[2], P[3])

            self.logHandler.helper.debug_log('RTL1: {} {} {} {}'.format(P[0], P[1], P[2], P[3]))
            return violation
        except Exception as e:
            self.logHandler.helper.debug_log("Error in RTL1: {}".format(e))
        finally:
            return violation


    def A_RTL2(self):
        if not self.logHandler.log_cols_ready():
            raise KeyError("Log Values not ready yet")
        
        # LHS:
        # mode == RTL
        # current alt >= RTL_ALT
        # current position != home position

        P = {}

        # P0: check mode if RTL
        if self.logHandler.opened_logs['MODE'][-1][1] == "RTL":
            P[0] = 1
        else:
            P[0] = -1

        # P1: current alt >= RTL_ALT
        rtl_alt = self.logHandler.opened_logs['RTL_ALT'][-1][1] * 10 // 1000 #convert to mm
        curr_alt = self.logHandler.opened_logs['relative_alt'][-1][1] // 1000
        if rtl_alt > 0:
            # P[1] = 1 if curr_alt > rtl_alt else -1
            # P[1] = (curr_alt/rtl_alt)-1

            # PGFUZZ implementation
            # The implementation here is also difference from the pg fuzz
            P[1] = (curr_alt - rtl_alt) / rtl_alt 
        else:
            # P[1] = -1
            P[1] = 0

        # P2: current position != home position
        home_lat = self.logHandler.opened_logs['lat'][0][1]
        home_lon = self.logHandler.opened_logs['lon'][0][1]
        curr_lat = self.logHandler.opened_logs['lat'][-1][1]
        curr_lon = self.logHandler.opened_logs['lon'][-1][1]
        P[2] = PolicyChecker.relative_displacement((home_lat, home_lon), (curr_lat, curr_lon), 1)

        # PGFuzz implementaiton
        P[2] = -1 if P[2] <= 0 else 1

        # RHS (to negate)
        # AND (
        #   current position != last position 
        #   maintain altitude
        # )

        # P3: current position == prev position
        prev_lat = self.get_prev('lat')
        prev_lon = self.get_prev('lon')
        curr_lat = self.logHandler.opened_logs['lat'][-1][1]
        curr_lon = self.logHandler.opened_logs['lon'][-1][1]
        P[3] = PolicyChecker.relative_proximity((prev_lat, prev_lon), (curr_lat, curr_lon), 1)
        
        # PGFuzz implementation
        P[3] = -1 if P[3] <= 0 else 1

        # P4: current alt != prev alt
        prev_alt = self.get_prev('relative_alt') // 1000 # convert to meter
        curr_alt = self.logHandler.opened_logs['relative_alt'][-1][1] // 1000  # convert to meter
        # P[4] = PolicyChecker.alt_change(prev_alt, curr_alt)
        # P[4] = abs(curr_alt-prev_alt)/prev_alt
        # P[4] = (prev_alt/curr_alt)-1
        
        # PGFuz implementation
        if prev_alt != 0 and prev_alt == curr_alt:
            P[4] = -1
        else:
            P[4] = PolicyChecker.alt_change(prev_alt, curr_alt)

        violation=min(P[0], P[1], P[2], max(P[3], P[4]))
        # if violation > 0:
        print('RTL2:', P[0], P[1], P[2], 'max', P[3], P[4])
        return violation

    def A_RTL3(self):
        if not self.logHandler.log_cols_ready():
            raise KeyError("Log Values not ready yet")
               
        # LHS:
        # mode == RTL/LAND
        # current alt >= RTL_ALT
        # current position == home position

        P = {}

        # P0: Check if mode is RTL
        if self.logHandler.opened_logs['MODE'][-1][1] == "RTL":
            P[0] = 1
        else:
            P[0] = -1

        # P2: current position == home position
        home_lat = self.logHandler.opened_logs['lat'][0][1]
        home_lon = self.logHandler.opened_logs['lon'][0][1]
        curr_lat = self.logHandler.opened_logs['lat'][-1][1]
        curr_lon = self.logHandler.opened_logs['lon'][-1][1]
        P[1] = PolicyChecker.relative_proximity((home_lat, home_lon), (curr_lat, curr_lon), 1) 

        # RHS (to negate)
        # current alt < previous alt
        
        #P3: current alt >= previous alt (negated)
        current_alt = self.logHandler.opened_logs['relative_alt'][-1][1]
        previous_alt = self.get_prev('relative_alt')
        # P[2] = PolicyChecker.not_descending(previous_alt, current_alt)
        P[2] = (current_alt-previous_alt)/previous_alt
        # P[2] = 1 if current_alt >= previous_alt else -1

        violation = min(P[0], P[1], P[2])
        # if violation > 0:
        print('RTL3:', P[0], P[1], P[2])

        return violation#, P[3]

    def A_RTL4(self):
        '''
        Parameters required:
            MODE
            relative_alt: Altitude relative to home, unit: mm
        '''
        if not self.logHandler.log_cols_ready():
            raise KeyError("Log Values not ready yet")
        
        P = {}
        
        # P0: previous mode == RTL and current mode == LAND
        current_mode = self.logHandler.opened_logs['MODE'][-1][1]
        previous_mode = self.get_prev_flight_mode(current_mode)

        if previous_mode == "RTL" and current_mode == "LAND":
            P[0] = 1
        else:
            P[0] = -1

        # P1: current alt == ground alt
        relative_alt = self.logHandler.opened_logs['relative_alt'][-1][1] // 1000
        if round(relative_alt, 1) == 0:
            P[1] = 1
        else:
            P[1] = 0

        # P2: Disarm == on
        from pymavlink_simulation import is_disarmed
        if is_disarmed:
            P[2] = -1
        else:
            P[2] = 1

        self.logHandler.helper.debug_log("RTL4: {} {} {}".format(P[0], P[1], P[2]))
        return min(P[0], P[1], P[2])


        # # LHS:
        # # mode == land/rtl
        # # current alt == ground alt

        # P = {}

        # # P0: mode == land/rtl
        # if 'MODE' in self.logHandler.opened_logs:
        #     if self.logHandler.opened_logs['MODE'][-1][1] == "RTL":
        #         P[0] = 1
        #     else:
        #         P[0] = -1
        # else:
        #     P[0] = -1

        # # P1: current alt == ground alt
        # try:
        #     if 'lat' in self.logHandler.opened_logs and 'lon' in self.logHandler.opened_logs:
        #         grnd_alt = self.logHandler.opened_logs['relative_alt'][0][1]
        #         curr_alt = self.logHandler.opened_logs['relative_alt'][-1][1]

        #         P[1] = - self.scalar_change(grnd_alt, curr_alt)
        #     else:
        #         P[1] = -1
        # except Exception as e:
        #     self.logHandler.helper.debug_log("Policy Checking Exception: " + str(e))
        #     P[1] = -1

        # # RHS (to negate)
        # # disarm == on

        # # P2: disarm != on
        # try:
        #     if 'armed' in self.logHandler.opened_logs:
        #         P[2] = self.logHandler.opened_logs['armed'][-1][1]
        #     else:
        #         P[2] = -1
        # except Exception as e:
        #     self.logHandler.helper.debug_log("Policy Checking Exception: " + str(e))
        #     P[2] = -1     
        
        # return min(P[0], P[1], P[2])
    
    def A_FLIP1(self):
        """
        Parameters required:
            MODE
            roll
            alt: MSL altitude, unit: mm
            relative_alt: Altitude relative to home, unit: mm
        """
        P = {}
        # P0: current flight mode = FLIP
        current_mode = self.logHandler.opened_logs['MODE'][-1][1]
        if current_mode == "FLIP":
            P[0] = 1
        else:
            P[0] = -1

        # P1: previous flight mode = ACRO/ALT_HOLD
        previous_mode = self.get_prev_flight_mode(current_mode)
        if previous_mode == "ACRO" or previous_mode == "ALT_HOLD":
            P[1] = -1
        else:
            P[1] = 1

        # P2: Roll_t <= 45
        roll = self.logHandler.opened_logs['roll'][-1][1]
        if abs(roll) <= 45:
            P[2] = -1
        else:
            P[2] = 1
        
        # P3: Throttle >= 1,500 (Yet, we can infer the actual throttle from current and previous altitudes)
        current_alt = self.logHandler.opened_logs['alt'][-1][1] // 1000
        previous_alt = self.get_prev('alt') // 1000

        if current_alt >= previous_alt:
            P[3] = -1
        else:
            P[3] = (previous_alt - current_alt) / previous_alt

        # P4: ALT_t >= 10
        current_relative_alt = self.logHandler.opened_logs['relative_alt'][-1][1] // 1000
        if current_relative_alt >= 10:
            P[4] = -1
        else:
            P[4] = (10 - current_relative_alt) / 10

        self.logHandler.helper.debug_log("Current flight mode: {}".format(current_mode))
        self.logHandler.helper.debug_log("FLIP1: {}, {} {} {} {}".format(P[0], P[1], P[2], P[3], P[4]))
        return min(P[0], max(P[1], P[2], P[3], P[4]))


    def A_FLIP2(self):
        """
        """
        pass
        P = {}
        # P0: current flight mode = FLIP
        current_mode = self.logHandler.opened_logs['MODE'][-1][1]
        if current_mode == "FLIP":
            P[0] = 1
        else:
            P[0] = -1

        # P1: -90 <= roll <= 45

    def A_FLIP3(self):
        """
        """
        if not self.logHandler.log_cols_ready():
            raise KeyError("Log Values not ready yet")
        
        P = {}

        # P0: previous flight mode = FLIP
        current_mode = self.logHandler.opened_logs['MODE'][-1][1]
        previous_mode = self.get_prev_flight_mode(current_mode)
        if previous_mode == "FLIP":
            P[0] = 1
        
            previous_roll = self.get_roll_before_flip() * 180 / math.pi
            current_roll = self.logHandler.opened_logs['roll'][-1][1] * 180 / math.pi

            # P1: current roll == previous roll
            if (round(current_roll, 0) == round(previous_roll, 0)):
                P[1] = -1
            else:
                P[1] = abs(previous_roll - current_roll) / previous_roll

            previous_pitch = self.get_pitch_before_flip() * 180 / math.pi
            current_pitch = self.logHandler.opened_logs['pitch'][-1][1] * 180 / math.pi
            # P2: current pitch == previous pitch
            if (round(current_pitch, 0) == round(previous_pitch, 0)):
                P[2] = -1
            else:
                P[2] = abs(previous_pitch - current_pitch) / previous_pitch

            previous_yaw = self.get_yaw_before_flip() * 180 / math.pi
            current_yaw = self.logHandler.opened_logs['yaw'][-1][1] * 180 / math.pi
            # P3: current yaw == previous yaw
            if abs(round(current_yaw, 0) - round(previous_yaw, 0)) < 20:
                P[3] = -1
            else:
                P[3] = abs(previous_yaw - current_yaw) / previous_yaw
        else:
            P[0] = -1
            P[1] = 0
            P[2] = 0
            P[3] = 0

        self.logHandler.helper.debug_log("FLIP3: {}, {} {} {}".format(P[0], P[1], P[2], P[3]))
        return min(P[0], max(P[1], P[2], P[3]))


    def A_FLIP4(self):
        """
        Parameters required:
            MODE
        """
        if not self.logHandler.log_cols_ready():
            raise KeyError("Log Values not ready yet")
        
        P = {}

        # P0: previous flight mode = FLIP
        current_mode = self.logHandler.opened_logs['MODE'][-1][1]
        previous_mode = self.get_prev_flight_mode(current_mode)
        if previous_mode == "FLIP":
            P[0] = 1

            filp_start_time = self.get_flip_start_time()
            current_time = self.logHandler.opened_logs['MODE'][-1][0]
            time_elapsed = (datetime.strptime(current_time, '%H:%M:%S.%f') - datetime.strptime(filp_start_time, '%H:%M:%S.%f')).seconds

            # P1: time elapsed <= 2.5
            if time_elapsed <= 2.5:
                P[1] = -1
            else:
                P[1] = (time_elapsed - 2.5) / 2.5
        else:
            P[0] = -1
            P[1] = 0

        self.logHandler.helper.debug_log("FLIP4: {}, {}".format(P[0], P[1]))
        return min(P[0], P[1])


    def ALT_HOLD1(self):
        '''
        Parameters required:
            EK2_ALT_SOURCE:  Primary altitude sensor source
            alt_series
            stable_counter
            alt_avg = alt_series / stable_counter
            alt_gps_avg
        '''

        if not self.logHandler.log_cols_ready():
            raise KeyError("Log Values not ready yet")

        P = {}

        # parameter init
        alt_source = self.logHandler.opened_logs["EK2_ALT_SOURCE"][-1][1] # Retrieve the source EK2_ALT_SOURCE from somewhere
        alt_series = 0
        stable_counter = 1
        alt_avg = alt_series / stable_counter
        alt_gps_avg = 1

        curr_barometer_alt = 0
        curr_gps_alt = 0

        # Check if using barometer
        if alt_source == 0:
            P[0] = 1
        else:
            P[0] = -1

        try:
            # Assuming not using alt_series
            curr_barometer_alt = self.logHandler.opened_logs["VFR_HUD_alt"][-1][1]
            curr_gps_alt = self.logHandler.opened_logs["alt"][-1][1]
            
            # ALT_t:('14:13:04.810', 587.1599731445312), ALT_GPS:('14:13:04.802', 587160)
            self.logHandler.helper.debug_log("[Debug] alt_source:{}, ALT_t:{}, ALT_GPS:{}".format(alt_source, curr_barometer_alt, curr_gps_alt))
            # print("[Debug] alt_source:{}, ALT_t:{}, ALT_GPS:{}".format(alt_source, curr_barometer_alt, curr_gps_alt))

            # Convert to meters
            curr_gps_alt /= 1000

        except Exception as e:
            self.logHandler.helper.debug_log("Policy Checking Exception in ALT_HOLD: " + str(e))
        
        if round(curr_barometer_alt, 1) != round(curr_gps_alt, 1):
            P[1] = -1
            P[2] = -1
        else:
            P[1] = 1
            P[2] = 1

        self.logHandler.helper.debug_log("[Debug] alt_source:{}, ALT_t:{}, ALT_GPS:{}".format(alt_source, curr_barometer_alt, curr_gps_alt))

        # print("[Debug] alt_source:%d, ALT_t:%f, ALT_GPS:%f" % (alt_source, round(curr_barometer_alt, 1), round(curr_gps_alt, 1)))

        return min(P[0], max(P[1], P[2]))


    def ALT_HOLD2(self):
        '''
        parameters:
            mode
            actual_throttle
            current_altitude
            previous_altitude
        '''
        if not self.logHandler.log_cols_ready():
            raise KeyError("Log Values not ready yet")

        P = {}

        # P0: previous flight mode = ALT_HOLD
        current_mode = self.logHandler.opened_logs['MODE'][-1][1]
        if current_mode == "ALT_HOLD":
            P[0] = 1
        else:
            P[0] = -1

        # P1: Throttle == 1500
        current_alt = self.logHandler.opened_logs['alt'][-1][1] // 1000
        previous_alt = self.get_prev('alt') // 1000

        if current_alt == previous_alt:
            P[1] = -1
        else:
            P[1] = abs((previous_alt - current_alt) / previous_alt)

        # P2: current alt == previous alt, same as P1 since throttle is 1500 means the drone is stable
        P[2] = P[1]

        return min(P[0], P[1], P[2])
    
    def A_CIRCLE1(self):
 
        self.logHandler.helper.debug_log("A_CIRCLE1 executed")

        if not self.logHandler.log_cols_ready():
            raise KeyError("Log values not ready yet")

        violation = -1          # pessimistic default
        try:
            P = {}              # sub-conditions / “predicates”

            # ------------------------------------------------------------------
            # P[0]  —  pitch stick is “up/forward”
            # ------------------------------------------------------------------
            # RC input is typically 1000-2000 µs; centre ≈1500.
            pitch_in = self.logHandler.opened_logs['RC_pitch'][-1][1]
            stick_up  = max(0, pitch_in - 1500)         # 0 when neutral/negative
            P[0] = stick_up / 500 if stick_up else -1   # normalise to 0-1

            # ------------------------------------------------------------------
            # P[1]  —  current radius still > 0 m
            # ------------------------------------------------------------------
            curr_radius = self.logHandler.opened_logs['orbit_radius'][-1][1]  # metres
            P[1] = 1 if curr_radius > 0 else -1

            # ------------------------------------------------------------------
            # P[2]  —  radius is strictly decreasing
            # ------------------------------------------------------------------
            prev_radius = self.get_prev('orbit_radius')
            if prev_radius is None:
                # first sample → cannot evaluate trend
                P[2] = 0
            elif prev_radius > 0:
                # positive value ⇒ how much it shrank relative to previous
                P[2] = (prev_radius - curr_radius) / prev_radius
            else:
                P[2] = 0                                 # already at/through 0

            # ------------------------------------------------------------------
            # Aggregate:  policy holds when *all* P[i] ≥ 0 and at least one is > 0.
            # We mirror your “min-then-max” pattern.
            # ------------------------------------------------------------------
            violation = max(min(P.values()), -1)

            self.logHandler.helper.debug_log(
                "PITCH_RADIUS: P0={:.2f} P1={} P2={:.2f}".format(P[0], P[1], P[2])
            )
            return violation

        except Exception as e:
            self.logHandler.helper.debug_log(f"Error in A_PITCH_RADIUS: {e}")
        finally:
            return violation

    def A_CIRCLE2(self):
        self.logHandler.helper.debug_log("A_CIRCLE2 executed")

        if not self.logHandler.log_cols_ready():
            raise KeyError("Log values not ready yet")

        violation = -1                       # pessimistic default
        try:
            P = {}                           # sub-predicates

            # ------------------------------------------------------------------
            # P[0] — stick is pulled back (pitch < 1500 µs)
            # ------------------------------------------------------------------
            pitch_in   = self.logHandler.opened_logs['RC_pitch'][-1][1]
            stick_down = max(0, 1500 - pitch_in)          # 0 if ≥ centre
            P[0] = stick_down / 500 if stick_down else -1 # –1 = not applicable

            # ------------------------------------------------------------------
            # P[1] — current radius still positive (there is something to widen)
            # ------------------------------------------------------------------
            curr_radius = self.logHandler.opened_logs['orbit_radius'][-1][1]
            P[1] = 1 if curr_radius > 0 else -1

            # ------------------------------------------------------------------
            # P[2] — radius is increasing
            # ------------------------------------------------------------------
            prev_radius = self.get_prev('orbit_radius')
            if prev_radius is None:
                P[2] = 0                       # first sample → trend unknown
            elif prev_radius > 0:
                P[2] = (curr_radius - prev_radius) / prev_radius
            else:
                P[2] = 0                       # already past zero → N/A

            # ------------------------------------------------------------------
            # Aggregate: rule satisfied only if *all* active predicates ≥0
            # ------------------------------------------------------------------
            violation = max(min(P.values()), -1)

            self.logHandler.helper.debug_log(
                "PITCH_RADIUS_EXPAND: P0={:.2f} P1={} P2={:.2f}"
                .format(P[0], P[1], P[2])
            )
            return violation

        except Exception as e:
            self.logHandler.helper.debug_log(f"Error in A_PITCH_RADIUS_EXPAND: {e}")
        finally:
            return violation

    def A_CIRCLE3(self):
   
        self.logHandler.helper.debug_log("A_CIRCLE3 executed")

        if not self.logHandler.log_cols_ready():
            raise KeyError("Log values not ready yet")

        violation = -1
        try:
            P = {}

            # P[0] — stick pushed right
            roll_in  = self.logHandler.opened_logs['RC_roll'][-1][1]          # µs
            stick_rt = max(0, roll_in - 1500)                                 # 0 if ≤ centre
            P[0] = stick_rt / 500 if stick_rt else -1                         # –1 = rule inactive

            # P[1] — vehicle actually turning clockwise (yaw-rate > 0)
            yaw_rate = self.logHandler.opened_logs['yaw_rate'][-1][1]         # deg/s
            P[1] = 1 if yaw_rate > 0 else -1

            # P[2] — groundspeed strictly increasing
            curr_spd = self.logHandler.opened_logs['ground_speed'][-1][1]     # m/s
            prev_spd = self.get_prev('ground_speed')
            if prev_spd is None:
                P[2] = 0            # first sample → trend unknown
            elif prev_spd > 0:
                P[2] = (curr_spd - prev_spd) / prev_spd
            else:
                P[2] = 0

            # aggregate
            violation = max(min(P.values()), -1)

            self.logHandler.helper.debug_log(
                f"A_CRICLE3: P0={P[0]:.2f} P1={P[1]} P2={P[2]:.2f}"
            )
            return violation

        except Exception as e:
            self.logHandler.helper.debug_log(f"Error in A_CRICLE3: {e}")
        finally:
            return violation

    def A_CIRCLE4(self):
  
        self.logHandler.helper.debug_log("A_CIRCLE4 executed")

        if not self.logHandler.log_cols_ready():
            raise KeyError("Log values not ready yet")

        violation = -1
        try:
            P = {}

            # ----------------------------------------------------------
            # P[0] — stick pushed right (clockwise roll command)
            # ----------------------------------------------------------
            roll_in  = self.logHandler.opened_logs['RC_roll'][-1][1]     # µs
            stick_rt = max(0, roll_in - 1500)                            # 0 if ≤ centre
            P[0] = stick_rt / 500 if stick_rt else -1                    # –1 ⇒ rule inactive

            # ----------------------------------------------------------
            # P[1] — vehicle turning counter-clockwise (yaw_rate < 0)
            # ----------------------------------------------------------
            yaw_rate = self.logHandler.opened_logs['yaw_rate'][-1][1]    # deg/s
            P[1] = 1 if yaw_rate < 0 else -1

            # ----------------------------------------------------------
            # P[2] — groundspeed strictly decreasing
            # ----------------------------------------------------------
            curr_spd = self.logHandler.opened_logs['ground_speed'][-1][1]  # m/s
            prev_spd = self.get_prev('ground_speed')
            if prev_spd is None:
                P[2] = 0                          # first sample → trend unknown
            elif prev_spd > 0:
                P[2] = (prev_spd - curr_spd) / prev_spd
            else:
                P[2] = 0

            # ----------------------------------------------------------
            # Aggregate:  rule holds when *all* active predicates ≥ 0
            # ----------------------------------------------------------
            violation = max(min(P.values()), -1)

            self.logHandler.helper.debug_log(
                f"A_CIRCLE4: P0={P[0]:.2f} P1={P[1]} P2={P[2]:.2f}"
            )
            return violation

        except Exception as e:
            self.logHandler.helper.debug_log(f"Error in A_CIRCLE4: {e}")
        finally:
            return violation

    def A_CIRCLE5(self):

        self.logHandler.helper.debug_log("A_CIRCLE5 executed")

        if not self.logHandler.log_cols_ready():
            raise KeyError("Log values not ready yet")

        violation = -1
        try:
            P = {}

            # ----------------------------------------------------------
            # P[0] — stick pushed left (roll < 1500 µs)
            # ----------------------------------------------------------
            roll_in   = self.logHandler.opened_logs['RC_roll'][-1][1]    # µs
            stick_lft = max(0, 1500 - roll_in)                           # 0 if ≥ centre
            P[0] = stick_lft / 500 if stick_lft else -1                  # –1 ⇒ rule inactive

            # ----------------------------------------------------------
            # P[1] — aircraft turning CCW (yaw_rate < 0)
            # ----------------------------------------------------------
            yaw_rate = self.logHandler.opened_logs['yaw_rate'][-1][1]    # deg/s
            P[1] = 1 if yaw_rate < 0 else -1

            # ----------------------------------------------------------
            # P[2] — ground-speed strictly increasing
            # ----------------------------------------------------------
            curr_spd = self.logHandler.opened_logs['ground_speed'][-1][1]   # m/s
            prev_spd = self.get_prev('ground_speed')
            if prev_spd is None:
                P[2] = 0                   # first sample → trend unknown
            elif prev_spd > 0:
                P[2] = (curr_spd - prev_spd) / prev_spd
            else:
                P[2] = 0

            # ----------------------------------------------------------
            # Aggregate:  rule holds only if *all* active predicates ≥ 0
            # ----------------------------------------------------------
            violation = max(min(P.values()), -1)

            self.logHandler.helper.debug_log(
                f"A_CIRCLE5: P0={P[0]:.2f} P1={P[1]} P2={P[2]:.2f}"
            )
            return violation

        except Exception as e:
            self.logHandler.helper.debug_log(f"Error in A_CIRCLE5: {e}")
        finally:
            return violation

    def A_CIRCLE6(self):
      
        self.logHandler.helper.debug_log("A_CIRCLE6 executed")

        if not self.logHandler.log_cols_ready():
            raise KeyError("Log values not ready yet")

        violation = -1
        try:
            P = {}

            # ----------------------------------------------------------
            # P[0] — stick pushed left (roll < 1500 µs)
            # ----------------------------------------------------------
            roll_in   = self.logHandler.opened_logs['RC_roll'][-1][1]      # µs
            stick_lft = max(0, 1500 - roll_in)                             # 0 if ≥ centre
            P[0] = stick_lft / 500 if stick_lft else -1                    # –1 ⇒ rule inactive

            # ----------------------------------------------------------
            # P[1] — vehicle turning clockwise (yaw_rate > 0)
            # ----------------------------------------------------------
            yaw_rate = self.logHandler.opened_logs['yaw_rate'][-1][1]      # deg/s
            P[1] = 1 if yaw_rate > 0 else -1

            # ----------------------------------------------------------
            # P[2] — ground-speed strictly decreasing
            # ----------------------------------------------------------
            curr_spd = self.logHandler.opened_logs['ground_speed'][-1][1]  # m/s
            prev_spd = self.get_prev('ground_speed')
            if prev_spd is None:
                P[2] = 0                        # first sample → trend unknown
            elif prev_spd > 0:
                P[2] = (prev_spd - curr_spd) / prev_spd
            else:
                P[2] = 0

            # ----------------------------------------------------------
            # Aggregate: rule holds only if *all* active predicates ≥ 0
            # ----------------------------------------------------------
            violation = max(min(P.values()), -1)

            self.logHandler.helper.debug_log(
                f"A_CIRCLE6: P0={P[0]:.2f} P1={P[1]} P2={P[2]:.2f}"
            )
            return violation

        except Exception as e:
            self.logHandler.helper.debug_log(f"Error in A_CIRCLE6: {e}")
        finally:
            return violation


    def A_CIRCLE7(self):
    
        self.logHandler.helper.debug_log("A_CIRCLE7 executed")

        if not self.logHandler.log_cols_ready():
            raise KeyError("Log values not ready yet")

        violation = -1
        try:
            P   = {}
            tol = 50          # ±50 µs band around 1500; tweak if you use %/-1…1

            # ----------------------------------------------------------
            # Helper: convert stick deviation into a predicate 0-1 range
            # ----------------------------------------------------------
            def stick_neutral(value):
                dev = abs(value - 1500)
                if dev <= tol:
                    return 1 - dev / tol       # 1 when dead-centre, 0 at band edge
                return -1                      # outside band ⇒ rule breached

            # Roll, Pitch, Yaw sticks must be neutral
            roll_in  = self.logHandler.opened_logs['RC_roll']  [-1][1]
            pitch_in = self.logHandler.opened_logs['RC_pitch'] [-1][1]
            yaw_in   = self.logHandler.opened_logs['RC_yaw']   [-1][1]

            P[0] = stick_neutral(roll_in)
            P[1] = stick_neutral(pitch_in)
            P[2] = stick_neutral(yaw_in)

            # ----------------------------------------------------------
            # Aggregate exactly like the earlier CIRCLE rules
            # ----------------------------------------------------------
            violation = max(min(P.values()), -1)

            self.logHandler.helper.debug_log(
                f"A_CIRCLE7: roll={P[0]:.2f} pitch={P[1]:.2f} yaw={P[2]:.2f}"
            )
            return violation

        except Exception as e:
            self.logHandler.helper.debug_log(f"Error in A_CIRCLE7: {e}")
        finally:
            return violation


    def A_LAND1(self):
        '''
        Parameters required:
            MODE
            RELATIVE_ALT: Altitude relative to home, unit: mm
            LAND_ALT_LOW: Altitude during Landing at which vehicle slows to LAND_SPEED, unit: cm
            LAND_SPEED_HIGH: Speed during landing, unit: cm/s
            WPNAV_SPEED_DN: Speed during landing, unit: cm/s
            vz: Speed during landing, unit: cm/s
        '''
        if not self.logHandler.log_cols_ready():
            raise KeyError("Log Values not ready yet")
        
        P = {}
        
        # P0: mode == LAND
        if self.logHandler.opened_logs['MODE'][-1][1] == "LAND":
            P[0] = 1
        else:
            P[0] = -1

        # P1: if current alt >= LAND_ALT_LOW (use 10 for now)
        current_alt = self.logHandler.opened_logs['relative_alt'][-1][1] // 1000 # convert to meter
        LAND_ALT_LOW = 10
        if current_alt == 0:
            P[1] = -1
        else:
            P[1] = 1

        # P2: speed vertical == LAND_SPEED_HIGH
        speed_vertical = self.logHandler.opened_logs['vz'][-1][1] # cm/s, positive down
        try:
            land_speed_high = self.logHandler.opened_logs['LAND_SPEED_HIGH'][-1][1] # cm/s
            # If 'LAND_SPEED_HIGH' configuration parameter is zero then WPNAV_SPEED_DN is used.
            if land_speed_high == 0:
                land_speed_high = self.logHandler.opened_logs['WPNAV_SPEED_DN'][-1][1]  # cm/s

            # The drone's actual landing speed cannot be exactly matched with the parameter value; thus, we leverage the following predicate.
            if abs(speed_vertical - land_speed_high) <= 50:
                P[2] = -1
            else:
                P[2] = abs(speed_vertical - land_speed_high) / land_speed_high
    
            self.logHandler.helper.debug_log("LAND1: {} {} {}".format(P[0], P[1], P[2]))

            return min(P[0], P[1], P[2])
        except Exception as e:
            self.logHandler.helper.debug_log("A_LAND1 exception: " + str(e))
            return -1

    def A_LAND2(self):
        '''
        Parameters required:
            MODE
            RELATIVE_ALT: Altitude relative to home, unit: mm
            LAND_ALT_LOW: Altitude during Landing at which vehicle slows to LAND_SPEED, unit: cm
            LAND_SPEED: The descent speed for the final stage of landing, unit: cm/s
            vz: Speed during landing, unit: cm/s
        '''
        if not self.logHandler.log_cols_ready():
            raise KeyError("Log Values not ready yet")
        
        P = {}
        
        # P0: mode == LAND
        if self.logHandler.opened_logs['MODE'][-1][1] == "LAND":
            P[0] = 1
        else:
            P[0] = -1

        # P1: if current alt >= LAND_ALT_LOW (use 10 for now)
        current_alt = self.logHandler.opened_logs['relative_alt'][-1][1] // 1000
        LAND_ALT_LOW = 10  # meter
        
        if current_alt >= LAND_ALT_LOW:
            P[1] = -1
        elif current_alt == 0:
            P[1] = -1  # Preventing false alrms when the drone lands on the ground
        else:
            P[1] = 1

        # P2: speed vertical == LAND_SPEED_HIGH
        # The drone's actual landing speed cannot be exactly matched with the parameter value; thus, we leverage the following predicate.
        speed_vertical = self.logHandler.opened_logs['vz'][-1][1]
        land_speed = self.logHandler.opened_logs['LAND_SPEED'][-1][1]
        if abs(speed_vertical - land_speed) <= 10:
            P[2] = -1
        else:
            P[2] = abs(speed_vertical - land_speed) / land_speed

        self.logHandler.helper.debug_log("LAND2: {} {} {}".format(P[0], P[1], P[2]))
        return min(P[0], P[1], P[2])

    def A_AUTO1(self):
        '''
        '''
        if not self.logHandler.log_cols_ready():
            raise KeyError("Log Values not ready yet")

        p = {}

        # P0: mode == AUTO
        if self.logHandler.opened_logs['MODE'][-1][1] == "AUTO":
            p[0] = 1
        else:
            p[0] = -1

        # P1: RC_yaw_t != 1500
        rc_yaw = self.logHandler.opened_logs['chan4_raw'][-1][1]
        if rc_yaw != 1500:
            p[1] = 1
        else:
            p[1] = -1

        # Yaw_t != Yaw_(t-1)
        yaw = self.logHandler.opened_logs['yaw'][-1][1]
        prev_yaw = self.get_prev('yaw')
        if yaw != prev_yaw:
            p[2] = -1
        else:
            p[2] = 1

        self.logHandler.helper.debug_log("AUTO1: {} {} {}".format(p[0], p[1], p[2]))
        return min(p[0],p[1], p[2])

    def A_RC_FS1(self):
    
        self.logHandler.helper.debug_log("A_RC_FS1 executed")

        if not self.logHandler.log_cols_ready():
            raise KeyError("Log values not ready yet")

        violation = -1
        try:
            P = {}

            # ------------------------------------------------------------------
            # Current telemetry -------------------------------------------------
            mode_cur   = self.logHandler.opened_logs['MODE']        [-1][1]  # e.g. 'ACRO'
            armed_cur  = self.logHandler.opened_logs['ARMED']       [-1][1]  # 1 = armed, 0 = disarmed
            thr_cur    = self.logHandler.opened_logs['RC_throttle'] [-1][1]  # µs (1000-2000) or %
            fs_thr_val = self.logHandler.opened_logs['FS_THR_VALUE'][-1][1]  # same units as thr_cur

            # ------------------------------------------------------------------
            # Predicate P0 : forward implication  (trigger ⇒ disarmed)
            # ------------------------------------------------------------------
            trigger = (mode_cur == "ACRO") and (thr_cur < fs_thr_val)

            if not trigger:
                P[0] = 1                     # rule vacuously true
            else:
                P[0] = 1 if armed_cur == 0 else -1

            # ------------------------------------------------------------------
            # Predicate P1 : reverse implication  (disarmed in ACRO ⇒ trigger)
            # ------------------------------------------------------------------
            if not (armed_cur == 0 and mode_cur == "ACRO"):
                P[1] = 1                     # reverse implication not active
            else:
                P[1] = 1 if thr_cur < fs_thr_val else -1

            # ------------------------------------------------------------------
            # Aggregate exactly like your other policies
            # ------------------------------------------------------------------
            violation = max(min(P.values()), -1)

            self.logHandler.helper.debug_log(
                f"A_RC_FS1: trigger={trigger}  P0={P[0]}  P1={P[1]}  →  {violation}"
            )
            return violation

        except Exception as e:
            self.logHandler.helper.debug_log(f"Error in A_RC_FS1: {e}")
        finally:
            return violation


    def A_RC_FS2(self):
        
        self.logHandler.helper.debug_log("A_RC_FS2 executed")

        if not self.logHandler.log_cols_ready():
            raise KeyError("Log values not ready yet")

        violation = -1
        try:
            # ------------------------------------------------------------------
            # Current telemetry -------------------------------------------------
            mode_cur   = self.logHandler.opened_logs['MODE']        [-1][1]  # string
            thr_cur    = self.logHandler.opened_logs['RC_throttle'] [-1][1]  # µs or %
            fs_thr_val = self.logHandler.opened_logs['FS_THR_VALUE'][-1][1]

            # ------------------------------------------------------------------
            # Configuration: which modes count as "RC fail-safe"?
            #   Adjust this list to match your firmware setup.
            # ------------------------------------------------------------------
            FAILSAFE_MODES = {"FS", "LAND", "RTL", "BRAKE"}

            # ------------------------------------------------------------------
            # Forward implication predicate
            # ------------------------------------------------------------------
            trigger = thr_cur < fs_thr_val
            in_fs   = mode_cur in FAILSAFE_MODES

            #  Discrete 1 / –1 score; replace with a gradient if you like
            P0 = 1 if (not trigger or in_fs) else -1

            violation = max(min(P0, 1), -1)

            self.logHandler.helper.debug_log(
                f"A_RC_FS2: trigger={trigger}  mode={mode_cur}  →  {violation}"
            )
            return violation

        except Exception as e:
            self.logHandler.helper.debug_log(f"Error in A_RC_FS2: {e}")
        finally:
            return violation


    def A_GPS_FS1(self):

        self.logHandler.helper.debug_log("A_GPS_FS1 executed")

        if not self.logHandler.log_cols_ready():
            raise KeyError("Log values not ready yet")

        violation = -1
        try:
            # --------------------------------------------------------------
            # Current telemetry
            # --------------------------------------------------------------
            n_sats   = self.logHandler.opened_logs['GPS_NSATS'][-1][1]   # satellites in view
            mode_cur = self.logHandler.opened_logs['MODE']      [-1][1]   # flight-mode string

            # --------------------------------------------------------------
            # Modes that count as “GPS failsafe”
            #   – tweak to match your firmware’s naming
            # --------------------------------------------------------------
            GPS_FAILSAFE_MODES = {"GPS_FS", "LAND", "BRAKE", "RTL"}

            trigger = n_sats < 4
            in_fs   = mode_cur in GPS_FAILSAFE_MODES

            P0 = 1 if (not trigger or in_fs) else -1      # forward implication

            violation = max(min(P0, 1), -1)

            self.logHandler.helper.debug_log(
                f"A_GPS_FS1: nsats={n_sats} trigger={trigger} mode={mode_cur} → {violation}"
            )
            return violation

        except Exception as e:
            self.logHandler.helper.debug_log(f"Error in A_GPS_FS1: {e}")
        finally:
            return violation


    def A_GPS_FS2(self):

        self.logHandler.helper.debug_log("A_GPS_FS2 executed")

        if not self.logHandler.log_cols_ready():
            raise KeyError("Log values not ready yet")

        violation = -1
        try:
            # --------------------------------------------------------------
            # Current telemetry -------------------------------------------
            # --------------------------------------------------------------
            mode_cur        = self.logHandler.opened_logs['MODE'][-1][1]       # string
            alt_sec_present = bool(self.logHandler.opened_logs['ALT_SEC_PRESENT'][-1][1])
            alt_primary_src = self.logHandler.opened_logs['ALT_PRIMARY_SRC'][-1][1]  # e.g. 'PRIMARY', 'SECONDARY'

            # --------------------------------------------------------------
            # Which flight-modes count as “GPS failsafe”?
            #   – adjust to match your firmware.
            # --------------------------------------------------------------
            GPS_FAILSAFE_MODES = {"GPS_FS", "LAND", "BRAKE", "RTL"}

            trigger = (mode_cur in GPS_FAILSAFE_MODES) and alt_sec_present
            in_sec  = (alt_primary_src == "SECONDARY")          # true if switch done

            # Forward implication predicate
            P0 = 1 if (not trigger or in_sec) else -1

            violation = max(min(P0, 1), -1)

            self.logHandler.helper.debug_log(
                f"A_GPS_FS2: trigger={trigger}  alt_primary={alt_primary_src}  →  {violation}"
            )
            return violation

        except Exception as e:
            self.logHandler.helper.debug_log(f"Error in A_GPS_FS2: {e}")
        finally:
            return violation


    def A_SPORT1(self):
        '''
        Params required:
            PILOT_SPEED_UP: cm/s
            vz: cm/s
        Notes:
            This is not a built in mode
        '''

        if not self.logHandler.log_cols_ready():
            raise KeyError("Log Values not ready yet")
        
        P = {}
        # P0: mode == SPORT
        if self.logHandler.opened_logs['MODE'][-1][1] == "SPORT":
            P[0] = 1
        else:
            P[0] = -1

        # P1: vertical_speed == PILOT_SPEED_UP
        vz = self.logHandler.opened_logs['vz'][-1][1]
        speed_up = self.logHandler.opened_logs['PILOT_SPEED_UP'][-1][1]
        if abs(vz - speed_up) <= 10:
            P[1] = -1
        else:
            P[1] = abs(vz - speed_up) / speed_up

        self.logHandler.helper.debug_log("SPORT1: {} {}".format(P[0], P[1]))

        return min(P[0], P[1])

    def A_GUIDED1(self):
        '''
        Skip this since we do not fuzz remote control
        '''
        pass

    def A_LOITER1(self):

        if not self.logHandler.log_cols_ready():
            raise KeyError("Log Values not ready yet")
        
        P = {}

        # P0: mode == LOITER
        if self.logHandler.opened_logs['MODE'][-1][1] == "LOITER":
            P[0] = 1
        else:
            P[0] = -1

        # P1: current_yaw == previous_yaw
        current_yaw = self.logHandler.opened_logs['yaw'][-1][1]
        previous_yaw = self.get_prev('yaw')
        if abs(current_yaw - previous_yaw) <= 10:
            P[1] = -1
        else:
            P[1] = abs(current_yaw - previous_yaw) / previous_yaw

        # P2: current_pos == previous_pos
        current_lat = self.logHandler.opened_logs['lat'][-1][1]
        current_lon = self.logHandler.opened_logs['lon'][-1][1]
        previous_lat = self.get_prev('lat')
        previous_lon = self.get_prev('lon')
        # self.logHandler.helper.debug_log("current_lat:{}, current_lon:{}, previous_lat:{}, previous_lon:{}".format(current_lat, current_lon, previous_lat, previous_lon))
        # self.logHandler.helper.debug_log(PolicyChecker.relative_displacement((current_lat, current_lon), (previous_lat, previous_lon)))
        if (PolicyChecker.relative_displacement((current_lat, current_lon), (previous_lat, previous_lon)) <= 0):
            P[2] = -1
        else:
            P[2] = PolicyChecker.relative_displacement((current_lat, current_lon), (previous_lat, previous_lon))
        
        # P3: current_alt == previous_alt
        current_alt = int(self.logHandler.opened_logs['relative_alt'][-1][1])
        previous_alt = int(self.get_prev('relative_alt'))
        # self.logHandler.helper.debug_log("current_alt:{}, previous_alt:{}".format(current_alt, previous_alt))
        delta = current_alt - previous_alt
        if delta <= 500:
            P[3] = -1
        else:
            P[3] = delta / previous_alt

        self.logHandler.helper.debug_log("LOITER1: {}, {} {} {}".format(P[0], P[1], P[2], P[3]))
        return min(P[0], max(P[1], P[2], P[3]))

    def A_DRIFT1(self):
        pass

    def A_BRAKE1(self):
        '''
        check if the drone 
        '''
        if not self.logHandler.log_cols_ready():
            raise KeyError("Log Values not ready yet")

        P = {}

        # P0: Mode_t = BRAKE
        current_mode = self.logHandler.opened_logs['MODE'][-1][1]
        if self.get_prev_flight_mode(current_mode) == "BRAKE":
            P[0] = 1
        else:
            P[0] = -1

        # P1: [0, k] POS_t = POS_(t-1)
        vx = self.logHandler.opened_logs['vx'][-1][1] // 10  # m/s
        vy = self.logHandler.opened_logs['vy'][-1][1] // 10  # m/s
        if vx == 0 and vy == 0:
            P[1] = -1
        else:
            P[1] = max(abs(vx), abs(vy))
        
        self.logHandler.helper.debug_log("BRAKE1: {} {}".format(P[0], P[1]))
        return min(P[0], P[1])