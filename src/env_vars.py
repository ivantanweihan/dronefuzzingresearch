# need to copy paste properly due to character encoding problem, python might not find the directory
HOME = "/home/waiwai/URP-Drone/"

ARDU_HOME = HOME + "ardupilot/"
PX4_HOME = HOME + "PX4-Autopilot/"
PROJ_HOME = HOME + "dronefuzzingresearch/"
QGC_HOME = PROJ_HOME + "bins/"
PX4_LOGS = PROJ_HOME + "px4_logs/"
ARDU_LOGS = PROJ_HOME + "ardu_logs/"
PCD_GEN = PROJ_HOME + "pcd_gen/"
MODELS = PROJ_HOME + "models/"

'''
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
                        # self.violation_scores['A.LOITER1'].append(loiter1)
                        # self.violation_scores['A.DRIFT1'].append(drift1)
                        # self.violation_scores['A.BRAKE1'].append(brake1)
        '''

violation_scores_blank = {
    # 'A.CHUTE1': [],
    # 'A.RTL1': [],
    # 'A.RTL2': [],
    # 'A.RTL3': [],
    # 'A.RTL4': [],
    # 'A.FLIP1': [],
    # 'A.FLIP2': [],
    # 'A.FLIP3': [],
    # 'A.FLIP4': [],
    # 'A.ALT_HOLD1': [],
    # 'A.ALT_HOLD2': [],
    # 'A.CIRCLE1': [],
    # 'A.CIRCLE2': [],
    # 'A.CIRCLE3': [],
    # 'A.CIRCLE4_6': [],
    # 'A.CIRCLE7': [],
    # 'A.LAND1': [],
    # 'A.LAND2': [],
    # 'A.AUTO1': [],
    # 'A.GPS_FS1': [],
    # 'A.GPS_FS2': [],
    # 'A.RC_FS1': [],
    # 'A.RC_FS2': [],
    # 'A.SPORT1': [],
    # 'A.GUIDED1': [],
    'A.LOITER1': [],
    # 'A.DRIFT1': [],
    # 'A.BRAKE1': [],
}

# Define geographical boundaries in lat, lon
polygon_corners = {
    'big': [
        (-35.36390088461317, 149.1644181341574), 
        (-35.362360972318854, 149.16443089296527),
        (-35.362236113277945, 149.16625540249393),
        (-35.3640933715847, 149.16588539706504)
    ],
    'small':[
        (-35.36309369978192, 149.16488481027423), 
        (-35.36359241254927, 149.16493174893125),
        (-35.36355304060069, 149.16566265087621),
        (-35.363051046573275, 149.16543332200908)
    ]
}

param_defaults = {
    ''' 
    - Pitch-axis: The drone tilts forward or backward.
    - Roll-axis: The drone tilts to the left or right side.
    - Yaw-axis: The drone revolves viewed from above counter or counterclockwise.
    '''
    
    # Position controller
    'PSC_POSXY_P': 1.0,  # Proportional gain for position control in the XY plane (horizontal position control).
    'PSC_VELXY_P': 2.0,  # Proportional gain for velocity control in the XY plane (horizontal velocity control).
    'PSC_POSZ_P': 1.0,   # Proportional gain for position control in the Z direction (altitude control).

    # Attitude controller
    'ATC_ANG_RLL_P': 4.5,  # Proportional gain for roll angle control (how strongly the roll angle is controlled).
    'ATC_RAT_RLL_I': 0.135,  # Integral gain for roll rate control (corrects steady-state errors in roll rate).
    'ATC_RAT_RLL_D': 0.003600,  # Derivative gain for roll rate control (reduces overshoot and oscillations in roll rate).
    'ATC_RAT_RLL_P': 0.135000,  # Proportional gain for roll rate control (how strongly the roll rate is controlled).
    'ATC_ANG_PIT_P': 4.5,  # Proportional gain for pitch angle control (how strongly the pitch angle is controlled).
    'ATC_RAT_PIT_P': 0.135000,  # Proportional gain for pitch rate control (how strongly the pitch rate is controlled).
    'ATC_RAT_PIT_I': 0.135000,  # Integral gain for pitch rate control (corrects steady-state errors in pitch rate).
    'ATC_RAT_PIT_D': 0.003600,  # Derivative gain for pitch rate control (reduces overshoot and oscillations in pitch rate).
    'ATC_ANG_YAW_P': 4.5,  # Proportional gain for yaw angle control (how strongly the yaw angle is controlled).
    'ATC_RAT_YAW_P': 0.09,  # Proportional gain for yaw rate control (how strongly the yaw rate is controlled).
    'ATC_RAT_YAW_I': 0.009,  # Integral gain for yaw rate control (corrects steady-state errors in yaw rate).
    'ATC_RAT_YAW_D': 0.0,  # Derivative gain for yaw rate control (reduces overshoot and oscillations in yaw rate, set to 0 indicating no derivative control).

    # Mission parameters
    'WPNAV_RADIUS': 200.0,  # Radius around a waypoint where the drone considers it has reached the waypoint.
    'WPNAV_SPEED': 1000.0,  # Speed setting for waypoint navigation (horizontal speed when navigating to a waypoint).
    'WPNAV_SPEED_DN': 150.0,  # Descent speed during waypoint navigation.
    'WPNAV_SPEED_UP': 250.0,  # Ascent speed during waypoint navigation.
    'WPNAV_ACCEL': 250.0,  # Horizontal acceleration during waypoint navigation.
    'WPNAV_ACCEL_Z': 100.0,  # Vertical acceleration during waypoint navigation.
    'WPNAV_JERK': 1.0,  # The rate of change of acceleration during waypoint navigation (how smoothly acceleration changes).
    'ANGLE_MAX': 3000,  # Maximum tilt angle in centidegrees (limits the tilt of the drone during aggressive maneuvers).

    # Return to Launch (RTL) parameters
    'RTL_CONE_SLOPE': 3.0,  # Slope of the return-to-launch (RTL) cone (affects how the drone ascends to the RTL altitude).
    'RTL_SPEED': 0,  # Speed during the return-to-launch (RTL) procedure (0 indicates the drone uses default or current speed).
    'RTL_ALT': 1500,  # Altitude to which the drone climbs before returning to the launch point (in centimeters).

    # New param
    # Parameter default values
    'BARO_GND_TEMP': 0,  # Ground temperature parameter (default value: 0)
    'BARO_ALT_OFFSET': 0,  # Ground altitude offset parameter (default value: 0)
    'BARO_PRIMARY': 0,  # Ground primary parameter (default value: 0)
    'PILOT_THR_FILT': 0,  # Pilot throttle filter parameter (default value: 0)
    'PILOT_TKOFF_ALT': 0,  # Pilot takeoff altitude parameter (default value: 0)
    'RTL_ALT': 1500,  # RTL altitude parameter (default value: 1500)
    'RTL_ALT_FINAL': 0,  # Final RTL altitude parameter (default value: 0)
    'PILOT_ACCEL_Z': 250,  # Pilot Z-axis acceleration parameter (default value: 250)
    'WP_NAVALT_MIN': 0,  # Minimum navigation altitude for waypoints (default value: 0)
    'LAND_ALT_LOW': 1000,  # Land altitude low parameter (default value: 1000)
    'ATC_ANGLE_BOOST': 1,  # Angle boost for attitude control (default value: 1)
    'ATC_ANG_LIM_TC': 1,  # Attitude control angle limit time constant (default value: 1)
    'BCN_ALT': 0,  # Beacon altitude parameter (default value: 0)
    'EK2_ALT_SOURCE': 0,  # EKF2 altitude source parameter (default value: 0)
    'EK2_ALT_M_NSE': 3,  # EKF2 altitude measurement noise parameter (default value: 3)
    'EK2_WIND_PSCALE': 0.5,  # EKF2 wind position scale (default value: 0.5)
    'EK3_ALT_M_NSE': 3,  # EKF3 altitude measurement noise parameter (default value: 3)
    'EK3_WIND_PSCALE': 0.5,  # EKF3 wind position scale (default value: 0.5)
    'FENCE_ALT_MAX': 100,  # Maximum fence altitude parameter (default value: 100)
    'PSC_POSZ_P': 1,  # Position controller Z-axis P gain (default value: 1)
    # 'SIM_FLOW_POS_Z': 0,  # Simulated flow position Z parameter (default value: 0)
    'GPS_POS1_Z': 0,  # GPS position 1 Z parameter (default value: 0)
    'GPS_POS2_Z': 0,  # GPS position 2 Z parameter (default value: 0)
    'RNGFND1_POS_Z': 0,  # Rangefinder 1 position Z parameter (default value: 0)
    'VISO_POS_Z': 0,  # Vision system position Z parameter (default value: 0)
    'EK2_VELD_M_NSE': 0.7,  # EKF2 velocity measurement noise parameter (default value: 0.7)
    'PILOT_THR_BHV': 0,  # Pilot throttle behavior parameter (default value: 0)
    'FS_THR_ENABLE': 1,  # Throttle failsafe enable parameter (default value: 1)
    'FS_THR_VALUE': 975,  # Throttle failsafe value parameter (default value: 975)
    'ACRO_THR_MID': 0,  # Acro mode throttle midpoint parameter (default value: 0)
    'ATC_THR_MIX_MIN': 0.1,  # Attitude control throttle mix minimum parameter (default value: 0.1)
    'ATC_THR_MIX_MAX': 0.5,  # Attitude control throttle mix maximum parameter (default value: 0.5)
    'ATC_THR_MIX_MAN': 0.1,  # Attitude control manual throttle mix parameter (default value: 0.1)
    'MOT_BOOST_SCALE': 0,  # Motor boost scale parameter (default value: 0)
    'MOT_SLEW_UP_TIME': 0,  # Motor slew up time parameter (default value: 0)
    'MOT_SLEW_DN_TIME': 0,  # Motor slew down time parameter (default value: 0)
    'ACRO_BAL_ROLL': 1,  # Acro balance roll parameter (default value: 1)
    'ACRO_RP_EXPO': 0.3,  # Acro roll/pitch expo parameter (default value: 0.3)
    'AHRS_TRIM_X': 0,  # AHRS trim X-axis parameter (default value: 0)
    'ATC_ACCEL_R_MAX': 110000,  # Attitude control acceleration R max parameter (default value: 110000)
    'ATC_ANG_RLL_P': 4.5,  # Attitude control roll angle P gain (default value: 4.5)
    'ATC_RATE_R_MAX': 0,  # Attitude control rate R max parameter (default value: 0)
    'ATC_RAT_RLL_P': 0.135,  # Attitude control roll rate P gain (default value: 0.135)
    'ATC_RAT_RLL_I': 0.135,  # Attitude control roll rate I gain (default value: 0.135)
    'ATC_RAT_RLL_IMAX': 0.5,  # Attitude control roll rate IMAX (default value: 0.5)
    'ATC_RAT_RLL_D': 0.0036,  # Attitude control roll rate D gain (default value: 0.0036)
    'ATC_RAT_RLL_FF': 0,  # Attitude control roll rate feedforward (default value: 0)
    'ATC_RAT_RLL_FLTT': 20,  # Attitude control roll rate FLTT (default value: 20)
    'ATC_RAT_RLL_FLTE': 0,  # Attitude control roll rate FLTE (default value: 0)
    'ATC_RAT_RLL_FLTD': 20,  # Attitude control roll rate FLTD (default value: 20)

}

state_ranges = {
    'vx': {
        'min': -500,
        'max': 500 # cm/s
    },
    'vy': {
        'min': -500,
        'max': 500 # cm/s
    },
    'vz': {
        'min': -500,
        'max': 500 # cm/s
    },
    'relative_alt': {
        'min': 0,
        'max': 600000 # mm
    },
    'RTL_ALT': {
        'min': 200, 
        'max': 300000
    },
    'alt_error': {
        'min': -1, 
        'max': 1
    },
    'airspeed': {
        'min': 0, 
        'max': 10
    },
    'chan1_raw': {  # Controls the roll
        'min': 0, 
        'max': 3000  # microseconds
    },
    'chan2_raw': {  # Controls the pitch
        'min': 0, 
        'max': 3000
    },
    'chan3_raw': {  # Controls the throttle
        'min': 0, 
        'max': 3000
    },
    'chan4_raw': {  # Controls the yaw
        'min': 0, 
        'max': 3000
    },
    'climb': {
        'min': 0, 
        'max': 10
    },
    'groundspeed': {
        'min': 0, 
        'max': 10
    },
    'heading': {
        'min': 0, 
        'max': 360
    },
    'lat': {
        'min': -90, 
        'max': 90
    },
    'lon': {
        'min': -180, 
        'max': 180
    },
    'nav_bearing': {  # 导航方位
        'min': -3.142, 
        'max': 3.142  # radians
    },
    'nav_pitch': {
        'min': -3.142, 
        'max': 3.142
    },
    'nav_roll': {
        'min': -3.142, 
        'max': 3.142
    },
    'pitch': {
        'min': -3.142, 
        'max': 3.142
    },
    'pitchspeed': {
        'min': -3.142, 
        'max': 3.142
    },
    'roll': {
        'min': -3.142, 
        'max': 3.142
    },
    'rollspeed': {
        'min': -3.142, 
        'max': 3.142
    },
    'satellites_visible': {
        'min': 0, 
        'max': 255
    },
    'target_bearing': {
        'min': 0, 
        'max': 360
    },
    'throttle': {
        'min': 0, 
        'max': 100
    },
    'wp_dist': {
        'min': 0, 
        'max': 50
    },
    'yaw': {
        'min': -3.142, 
        'max': 3.142
    },
    'yawspeed': {
        'min': -3.142, 
        'max': 3.142
    }
}

params_to_exclude = [
    'RCMAP_PITCH', 'RCMAP_THROTTLE', 'RCMAP_ROLL', 'RCMAP_YAW', 'RTL_ALT_FINAL', 'RTL_LOIT_TIME', 'RTL_ALT'
    ]

simulation_vars = {
    'set_wait_secs': 3,
    # 'A.CHUTE1': {
    #     'hang_thres': 420,
    #     # 'hang_thres': 60, # test run
    #     'waypoints': 3,
    #     'wp_altitude': [30, 50],
    #     'params_to_fetch': ['RTL_ALT'],
    #     'params_to_fuzz': ['WPNAV_SPEED','WPNAV_RADIUS','WPNAV_SPEED_UP','WPNAV_SPEED_DN', 'WPNAV_ACCEL', 'WPNAV_ACCEL_Z', 'WPNAV_JERK', 'ANGLE_MAX'],
    #     'polygon_corners': polygon_corners['big'],
    #     'modes_vec': ['STABILIZE', 'AUTO', 'RTL'],
    #     'log_cols': ['MODE', 'lat', 'lon', 'relative_alt','roll','pitch','yaw','rollspeed', 'pitchspeed', 'yawspeed','xacc','yacc','zacc','xgyro','ygyro','zgyro', 'vx', 'vy', 'vz'],
    #     'state_cols': ['vx','vy','vz', 'relative_alt','RTL_ALT'],
    #     'constant_params': {},
    # },
    'A.RTL1': {
        'hang_thres': 420,
        # 'hang_thres': 60, # test run
        'wp_altitude': [30, 50],
        'waypoints': 3,
        'params_to_fetch': ['RTL_ALT'],
        'params_to_fuzz': ['WPNAV_SPEED','WPNAV_RADIUS','WPNAV_SPEED_UP','WPNAV_SPEED_DN', 'WPNAV_ACCEL', 'WPNAV_ACCEL_Z', 'WPNAV_JERK', 'ANGLE_MAX'],
        'polygon_corners': polygon_corners['big'],
        'modes_vec': ['STABILIZE', 'AUTO', 'RTL'],
        'log_cols': ['MODE', 'lat', 'lon', 'relative_alt','roll','pitch','yaw','rollspeed', 'pitchspeed', 'yawspeed','xacc','yacc','zacc','xgyro','ygyro','zgyro', 'vx', 'vy', 'vz'],
        'state_cols': ['vx','vy','vz', 'relative_alt','RTL_ALT'],
        'constant_params': {},
    }, 
    'A.RTL2': {
        'hang_thres': 420,
        # 'hang_thres': 60, # test run
        'waypoints': 3,
        'wp_altitude': [20, 25],
        'polygon_corners': polygon_corners['big'],
        # 'params_to_fuzz': ['WPNAV_SPEED','WPNAV_RADIUS','WPNAV_SPEED_UP','WPNAV_SPEED_DN', 'WPNAV_ACCEL', 'WPNAV_ACCEL_Z', 'WPNAV_JERK', 'ANGLE_MAX'],
        # 'params_to_fuzz': ["BARO_GND_TEMP", "BARO_ALT_OFFSET", "BARO_PRIMARY", "PILOT_THR_FILT", "PILOT_TKOFF_ALT", "RTL_ALT", "RTL_ALT_FINAL", "PILOT_ACCEL_Z", "WP_NAVALT_MIN", "LAND_ALT_LOW", "ATC_ANGLE_BOOST", "ATC_ANG_LIM_TC", "BCN_ALT", "EK2_ALT_SOURCE", "EK2_ALT_M_NSE", "EK2_WIND_PSCALE", "EK3_ALT_M_NSE", "EK3_WIND_PSCALE", "FENCE_ALT_MAX", "PSC_POSZ_P", "GPS_POS1_Z", "GPS_POS2_Z", "RNGFND1_POS_Z", "VISO_POS_Z", "EK2_VELD_M_NSE", "PILOT_THR_FILT", "PILOT_THR_BHV", "FS_THR_ENABLE", "FS_THR_VALUE", "ACRO_THR_MID", "ATC_THR_MIX_MIN", "ATC_THR_MIX_MAX", "ATC_THR_MIX_MAN", "MOT_BOOST_SCALE", "MOT_SLEW_UP_TIME", "MOT_SLEW_DN_TIME", "ACRO_BAL_ROLL", "ACRO_RP_EXPO", "AHRS_TRIM_X", "ATC_ACCEL_R_MAX", "ATC_ANG_RLL_P", "ATC_RATE_R_MAX", "ATC_RAT_RLL_P", "ATC_RAT_RLL_I", "ATC_RAT_RLL_IMAX", "ATC_RAT_RLL_D", "ATC_RAT_RLL_FF", "ATC_RAT_RLL_FLTT", "ATC_RAT_RLL_FLTE", "ATC_RAT_RLL_FLTD", "ATC_RAT_RLL_P", "ATC_RAT_RLL_I", "ATC_RAT_RLL_IMAX", "ATC_RAT_RLL_D", "ATC_RAT_RLL_FLTT", "ATC_RAT_RLL_FLTE", "ATC_RAT_RLL_FLTD"],
        'params_to_fuzz': ["PILOT_THR_FILT", "PILOT_TKOFF_ALT", "RTL_ALT", "RTL_ALT_FINAL", "PILOT_ACCEL_Z", "LAND_ALT_LOW", "ATC_ANG_LIM_TC"],
        'params_to_fetch': ['RTL_ALT'],
        'modes_vec': ['STABILIZE', 'AUTO', 'RTL'],
        'log_cols': ['MODE', 'lat', 'lon', 'relative_alt','roll','pitch','yaw','rollspeed', 'pitchspeed', 'yawspeed','xacc','yacc','zacc','xgyro','ygyro','zgyro', 'vx', 'vy', 'vz'],
        'state_cols': ['vx','vy','vz', 'relative_alt','RTL_ALT'],
        'constant_params': {},
    }, 
    # 'A.RTL3': {
    #     'hang_thres': 420,
    #     # 'hang_thres': 60, # test run
    #     'waypoints': 3,
    #     'wp_altitude': [20, 25],
    #     'polygon_corners': polygon_corners['big'],
    #     'params_to_fuzz': ['WPNAV_SPEED','WPNAV_RADIUS','WPNAV_SPEED_UP','WPNAV_SPEED_DN', 'WPNAV_ACCEL', 'WPNAV_ACCEL_Z', 'WPNAV_JERK', 'ANGLE_MAX'],
    #     'params_to_fetch': ['RTL_ALT'],
    #     'modes_vec': ['STABILIZE', 'AUTO', 'RTL'],
    #     'log_cols': ['MODE', 'lat', 'lon', 'relative_alt','roll','pitch','yaw','rollspeed', 'pitchspeed', 'yawspeed','xacc','yacc','zacc','xgyro','ygyro','zgyro', 'vx', 'vy', 'vz'],
    #     'state_cols': ['vx','vy','vz', 'relative_alt','RTL_ALT'],
    #     'constant_params': {},
    # },
    'A.RTL4': {
        'hang_thres': 420,
        # 'hang_thres': 60, # test run
        'waypoints': 3,
        'wp_altitude': [20, 25],
        'polygon_corners': polygon_corners['big'],
        'params_to_fuzz': ['WPNAV_SPEED','WPNAV_RADIUS','WPNAV_SPEED_UP','WPNAV_SPEED_DN', 'WPNAV_ACCEL', 'WPNAV_ACCEL_Z', 'WPNAV_JERK', 'ANGLE_MAX'],
        'params_to_fetch': ['RTL_ALT'],
        'modes_vec': ['STABILIZE', 'AUTO', 'RTL'],
        'log_cols': ['MODE', 'lat', 'lon', 'relative_alt','roll','pitch','yaw','rollspeed', 'pitchspeed', 'yawspeed','xacc','yacc','zacc','xgyro','ygyro','zgyro', 'vx', 'vy', 'vz'],
        'state_cols': ['vx','vy','vz', 'relative_alt','RTL_ALT'],
        'constant_params': {},
    },
    'A.FLIP1': {
        'hang_thres': 420,
        # 'hang_thres': 60, # test run
        'wp_altitude': [30, 50],
        'waypoints': 3,
        'params_to_fetch': [],
        'params_to_fuzz': ['WPNAV_SPEED','WPNAV_RADIUS','WPNAV_SPEED_UP','WPNAV_SPEED_DN', 'WPNAV_ACCEL', 'WPNAV_ACCEL_Z', 'WPNAV_JERK', 'ANGLE_MAX'],
        'polygon_corners': polygon_corners['big'],
        'modes_vec': ['STABILIZE', 'RTL', 'ACRO', 'ALTHOLD', 'FLIP'],
        'log_cols': ['MODE', 'lat', 'lon', 'relative_alt','roll','pitch','yaw','rollspeed', 'pitchspeed', 'yawspeed','xacc','yacc','zacc','xgyro','ygyro','zgyro', 'vx', 'vy', 'vz', 'alt', 'chan4_raw'],
        'state_cols': ['vx','vy','vz', 'relative_alt','RTL_ALT'],
        'constant_params': {},
    },
        # 'A.FLIP2': {
    #     'hang_thres': 420,
    #     # 'hang_thres': 60, # test run
    #     'wp_altitude': [30, 50],
    #     'waypoints': 3,
    #     'params_to_fetch': ['RTL_ALT'],
    #     'params_to_fuzz': ['WPNAV_SPEED','WPNAV_RADIUS','WPNAV_SPEED_UP','WPNAV_SPEED_DN', 'WPNAV_ACCEL', 'WPNAV_ACCEL_Z', 'WPNAV_JERK', 'ANGLE_MAX'],
    #     'polygon_corners': polygon_corners['big'],
    #     'modes_vec': ['STABILIZE', 'AUTO', 'RTL'],
    #     'log_cols': ['MODE', 'lat', 'lon', 'relative_alt','roll','pitch','yaw','rollspeed', 'pitchspeed', 'yawspeed','xacc','yacc','zacc','xgyro','ygyro','zgyro', 'vx', 'vy', 'vz'],
    #     'state_cols': ['vx','vy','vz', 'relative_alt','RTL_ALT'],
    #     'constant_params': {},
    # },
    'A.FLIP3': {
        'hang_thres': 420,
        # 'hang_thres': 60, # test run
        'wp_altitude': [30, 50],
        'waypoints': 3,
        'params_to_fetch': ['RTL_ALT'],
        'params_to_fuzz': ['WPNAV_SPEED','WPNAV_RADIUS','WPNAV_SPEED_UP','WPNAV_SPEED_DN', 'WPNAV_ACCEL', 'WPNAV_ACCEL_Z', 'WPNAV_JERK', 'ANGLE_MAX'],
        'polygon_corners': polygon_corners['big'],
        'modes_vec': ['STABILIZE', 'AUTO', 'RTL'],
        'log_cols': ['MODE', 'lat', 'lon', 'relative_alt','roll','pitch','yaw','rollspeed', 'pitchspeed', 'yawspeed','xacc','yacc','zacc','xgyro','ygyro','zgyro', 'vx', 'vy', 'vz'],
        'state_cols': ['vx','vy','vz', 'relative_alt','RTL_ALT'],
        'constant_params': {},
    },
    'A.FLIP4': {
        'hang_thres': 420,
        # 'hang_thres': 60, # test run
        'wp_altitude': [30, 50],
        'waypoints': 3,
        'params_to_fetch': ['RTL_ALT'],
        'params_to_fuzz': ['WPNAV_SPEED','WPNAV_RADIUS','WPNAV_SPEED_UP','WPNAV_SPEED_DN', 'WPNAV_ACCEL', 'WPNAV_ACCEL_Z', 'WPNAV_JERK', 'ANGLE_MAX'],
        'polygon_corners': polygon_corners['big'],
        'modes_vec': ['STABILIZE', 'AUTO', 'RTL'],
        'log_cols': ['MODE', 'lat', 'lon', 'relative_alt','roll','pitch','yaw','rollspeed', 'pitchspeed', 'yawspeed','xacc','yacc','zacc','xgyro','ygyro','zgyro', 'vx', 'vy', 'vz'],
        'state_cols': ['vx','vy','vz', 'relative_alt','RTL_ALT'],
        'constant_params': {},
    },
    # 'A.ALT_HOLD1': {
    #     'hang_thres': 420,
    #     # 'hang_thres': 60, # test run
    #     'wp_altitude': [30, 50],
    #     'waypoints': 3,
    #     'params_to_fetch': ['EK2_ALT_SOURCE', 'RTL_ALT'],
    #     'params_to_fuzz': ['WPNAV_SPEED','WPNAV_RADIUS','WPNAV_SPEED_UP','WPNAV_SPEED_DN', 'WPNAV_ACCEL', 'WPNAV_ACCEL_Z', 'WPNAV_JERK', 'ANGLE_MAX'],
    #     'polygon_corners': polygon_corners['big'],
    #     'modes_vec': ['STABILIZE', 'AUTO', 'RTL'],
    #     'log_cols': ['MODE', 'lat', 'lon', 'relative_alt','roll','pitch','yaw','rollspeed', 'pitchspeed', 'yawspeed','xacc','yacc','zacc','xgyro','ygyro','zgyro', 'vx', 'vy', 'vz', 'alt', 'VFR_HUD_alt'],
    #     'state_cols': ['vx','vy','vz', 'relative_alt','RTL_ALT'],
    #     'constant_params': {},
    # }, 
        # 'A.ALT_HOLD2': {
    #     'hang_thres': 420,
    #     # 'hang_thres': 60, # test run
    #     'wp_altitude': [30, 50],
    #     'waypoints': 3,
    #     'params_to_fetch': ['EK2_ALT_SOURCE', 'RTL_ALT'],
    #     'params_to_fuzz': ['WPNAV_SPEED','WPNAV_RADIUS','WPNAV_SPEED_UP','WPNAV_SPEED_DN', 'WPNAV_ACCEL', 'WPNAV_ACCEL_Z', 'WPNAV_JERK', 'ANGLE_MAX'],
    #     'polygon_corners': polygon_corners['big'],
    #     'modes_vec': ['STABILIZE', 'AUTO', 'RTL'],
    #     'log_cols': ['MODE', 'lat', 'lon', 'relative_alt','roll','pitch','yaw','rollspeed', 'pitchspeed','yawspeed','xacc','yacc','zacc','xgyro','ygyro','zgyro', 'vx', 'vy', 'vz', 'alt', 'VFR_HUD_alt'],
    #     'state_cols': ['vx','vy','vz', 'relative_alt','RTL_ALT'],
    #     'constant_params': {},
    # },
    # 'A.CIRCLE1': {
    #     'hang_thres': 420,
    #     'waypoints': 3,
    #     'wp_altitude': [30, 50],
    #     'params_to_fetch': ['RTL_ALT'],
    #     'params_to_fuzz': ['WPNAV_SPEED','WPNAV_RADIUS','WPNAV_SPEED_UP','WPNAV_SPEED_DN', 'WPNAV_ACCEL', 'WPNAV_ACCEL_Z', 'WPNAV_JERK', 'ANGLE_MAX'],
    #     'polygon_corners': polygon_corners['big'],
    #     'modes_vec': ['STABILIZE', 'AUTO', 'RTL'],
    #     'log_cols': ['MODE', 'lat', 'lon', 'relative_alt','roll','pitch','yaw','rollspeed', 'pitchspeed','yawspeed','xacc','yacc','zacc','xgyro','ygyro','zgyro', 'vx', 'vy', 'vz'],
    #     'state_cols': ['vx','vy','vz', 'relative_alt','RTL_ALT'],
    #     'constant_params': {},
    # },
    # 'A.CIRCLE2': {
    #     'hang_thres': 420,
    #     'waypoints': 3,
    #     'wp_altitude': [30, 50],
    #     'params_to_fetch': ['RTL_ALT'],
    #     'params_to_fuzz': ['WPNAV_SPEED','WPNAV_RADIUS','WPNAV_SPEED_UP','WPNAV_SPEED_DN', 'WPNAV_ACCEL', 'WPNAV_ACCEL_Z', 'WPNAV_JERK', 'ANGLE_MAX'],
    #     'polygon_corners': polygon_corners['big'],
    #     'modes_vec': ['STABILIZE', 'AUTO', 'RTL'],
    #     'log_cols': ['MODE', 'lat', 'lon', 'relative_alt','roll','pitch','yaw','rollspeed', 'pitchspeed','yawspeed','xacc','yacc','zacc','xgyro','ygyro','zgyro', 'vx', 'vy', 'vz'],
    #     'state_cols': ['vx','vy','vz', 'relative_alt','RTL_ALT'],
    #     'constant_params': {},
    # },
    # 'A.CIRCLE3': {
    #     'hang_thres': 420,
    #     'waypoints': 3,
    #     'wp_altitude': [30, 50],
    #     'params_to_fetch': ['RTL_ALT'],
    #     'params_to_fuzz': ['WPNAV_SPEED','WPNAV_RADIUS','WPNAV_SPEED_UP','WPNAV_SPEED_DN', 'WPNAV_ACCEL', 'WPNAV_ACCEL_Z', 'WPNAV_JERK', 'ANGLE_MAX'],
    #     'polygon_corners': polygon_corners['big'],
    #     'modes_vec': ['STABILIZE', 'AUTO', 'RTL'],
    #     'log_cols': ['MODE', 'lat', 'lon', 'relative_alt','roll','pitch','yaw','rollspeed', 'pitchspeed','yawspeed','xacc','yacc','zacc','xgyro','ygyro','zgyro', 'vx', 'vy', 'vz'],
    #     'state_cols': ['vx','vy','vz', 'relative_alt','RTL_ALT'],
    #     'constant_params': {},
    # },
    # 'A.CIRCLE4_6': {
    #     'hang_thres': 420,
    #     'waypoints': 3,
    #     'wp_altitude': [30, 50],
    #     'params_to_fetch': ['RTL_ALT'],
    #     'params_to_fuzz': ['WPNAV_SPEED','WPNAV_RADIUS','WPNAV_SPEED_UP','WPNAV_SPEED_DN', 'WPNAV_ACCEL', 'WPNAV_ACCEL_Z', 'WPNAV_JERK', 'ANGLE_MAX'],
    #     'polygon_corners': polygon_corners['big'],
    #     'modes_vec': ['STABILIZE', 'AUTO', 'RTL'],
    #     'log_cols': ['MODE', 'lat', 'lon', 'relative_alt','roll','pitch','yaw','rollspeed', 'pitchspeed','yawspeed','xacc','yacc','zacc','xgyro','ygyro','zgyro', 'vx', 'vy', 'vz'],
    #     'state_cols': ['vx','vy','vz', 'relative_alt','RTL_ALT'],
    #     'constant_params': {},
    # },
    # 'A.CIRCLE7': {
    #     'hang_thres': 420,
    #     'waypoints': 3,
    #     'wp_altitude': [30, 50],
    #     'params_to_fetch': ['RTL_ALT'],
    #     'params_to_fuzz': ['WPNAV_SPEED','WPNAV_RADIUS','WPNAV_SPEED_UP','WPNAV_SPEED_DN', 'WPNAV_ACCEL', 'WPNAV_ACCEL_Z', 'WPNAV_JERK', 'ANGLE_MAX'],
    #     'polygon_corners': polygon_corners['big'],
    #     'modes_vec': ['STABILIZE', 'AUTO', 'RTL'],
    #     'log_cols': ['MODE', 'lat', 'lon', 'relative_alt','roll','pitch','yaw','rollspeed', 'pitchspeed','yawspeed','xacc','yacc','zacc','xgyro','ygyro','zgyro', 'vx', 'vy', 'vz'],
    #     'state_cols': ['vx','vy','vz', 'relative_alt','RTL_ALT'],
    #     'constant_params': {},
    # },
    'A.LAND1': {
        'hang_thres': 420,
        'waypoints': 3,
        'wp_altitude': [30, 50],
        'params_to_fetch': ['LAND_SPEED_HIGH', 'WPNAV_SPEED_DN', 'RTL_ALT'],
        'params_to_fuzz': ['WPNAV_SPEED','WPNAV_RADIUS','WPNAV_SPEED_UP','WPNAV_SPEED_DN', 'WPNAV_ACCEL', 'WPNAV_ACCEL_Z', 'WPNAV_JERK', 'ANGLE_MAX'],
        'polygon_corners': polygon_corners['big'],
        'modes_vec': ['LAND'],
        'log_cols': ['MODE', 'lat', 'lon', 'relative_alt','roll','pitch','yaw','rollspeed', 'pitchspeed','yawspeed','xacc','yacc','zacc','xgyro','ygyro','zgyro', 'vx', 'vy', 'vz'],
        'state_cols': ['vx','vy','vz', 'relative_alt'],
        'constant_params': {},
    },
    'A.LAND2': {
        'hang_thres': 420,
        'waypoints': 3,
        'wp_altitude': [30, 50],
        'params_to_fetch': ['RTL_ALT', 'LAND_SPEED'],
        'params_to_fuzz': ['WPNAV_SPEED','WPNAV_RADIUS','WPNAV_SPEED_UP','WPNAV_SPEED_DN', 'WPNAV_ACCEL', 'WPNAV_ACCEL_Z', 'WPNAV_JERK', 'ANGLE_MAX'],
        'polygon_corners': polygon_corners['big'],
        'modes_vec': ['STABILIZE', 'AUTO', 'RTL'],
        'log_cols': ['MODE', 'lat', 'lon', 'relative_alt','roll','pitch','yaw','rollspeed', 'pitchspeed','yawspeed','xacc','yacc','zacc','xgyro','ygyro','zgyro', 'vx', 'vy', 'vz'],
        'state_cols': ['vx','vy','vz', 'relative_alt','RTL_ALT'],
        'constant_params': {},
    },
    # 'A.AUTO1': {
    #     'hang_thres': 420,
    #     'waypoints': 3,
    #     'wp_altitude': [30, 50],
    #     'params_to_fetch': ['RTL_ALT'],
    #     'params_to_fuzz': ['WPNAV_SPEED','WPNAV_RADIUS','WPNAV_SPEED_UP','WPNAV_SPEED_DN', 'WPNAV_ACCEL', 'WPNAV_ACCEL_Z', 'WPNAV_JERK', 'ANGLE_MAX'],
    #     'polygon_corners': polygon_corners['big'],
    #     'modes_vec': ['STABILIZE', 'AUTO', 'RTL'],
    #     'log_cols': ['MODE', 'lat', 'lon', 'relative_alt','roll','pitch','yaw','rollspeed', 'pitchspeed','yawspeed','xacc','yacc','zacc','xgyro','ygyro','zgyro', 'vx', 'vy', 'vz', ''],
    #     'state_cols': ['vx','vy','vz', 'relative_alt','RTL_ALT'],
    #     'constant_params': {},
    # },
    # 'A.GPS_FS1': {
    #     'hang_thres': 420,
    #     'waypoints': 3,
    #     'wp_altitude': [30, 50],
    #     'params_to_fetch': ['RTL_ALT'],
    #     'params_to_fuzz': ['WPNAV_SPEED','WPNAV_RADIUS','WPNAV_SPEED_UP','WPNAV_SPEED_DN', 'WPNAV_ACCEL', 'WPNAV_ACCEL_Z', 'WPNAV_JERK', 'ANGLE_MAX'],
    #     'polygon_corners': polygon_corners['big'],
    #     'modes_vec': ['STABILIZE', 'AUTO', 'RTL'],
    #     'log_cols': ['MODE', 'lat', 'lon', 'relative_alt','roll','pitch','yaw','rollspeed', 'pitchspeed','yawspeed','xacc','yacc','zacc','xgyro','ygyro','zgyro', 'vx', 'vy', 'vz'],
    #     'state_cols': ['vx','vy','vz', 'relative_alt','RTL_ALT'],
    #     'constant_params': {},
    # },
    # 'A.GPS_FS2': {
    #     'hang_thres': 420,
    #     'waypoints': 3,
    #     'wp_altitude': [30, 50],
    #     'params_to_fetch': ['RTL_ALT'],
    #     'params_to_fuzz': ['WPNAV_SPEED','WPNAV_RADIUS','WPNAV_SPEED_UP','WPNAV_SPEED_DN', 'WPNAV_ACCEL', 'WPNAV_ACCEL_Z', 'WPNAV_JERK', 'ANGLE_MAX'],
    #     'polygon_corners': polygon_corners['big'],
    #     'modes_vec': ['STABILIZE', 'AUTO', 'RTL'],
    #     'log_cols': ['MODE', 'lat', 'lon', 'relative_alt','roll','pitch','yaw','rollspeed', 'pitchspeed','yawspeed','xacc','yacc','zacc','xgyro','ygyro','zgyro', 'vx', 'vy', 'vz'],
    #     'state_cols': ['vx','vy','vz', 'relative_alt','RTL_ALT'],
    #     'constant_params': {},
    # },
    # 'A.RC_FS1': {
    #     'hang_thres': 420,
    #     'waypoints': 3,
    #     'wp_altitude': [30, 50],
    #     'params_to_fetch': ['RTL_ALT'],
    #     'params_to_fuzz': ['WPNAV_SPEED','WPNAV_RADIUS','WPNAV_SPEED_UP','WPNAV_SPEED_DN', 'WPNAV_ACCEL', 'WPNAV_ACCEL_Z', 'WPNAV_JERK', 'ANGLE_MAX'],
    #     'polygon_corners': polygon_corners['big'],
    #     'modes_vec': ['STABILIZE', 'AUTO', 'RTL'],
    #     'log_cols': ['MODE', 'lat', 'lon', 'relative_alt','roll','pitch','yaw','rollspeed', 'pitchspeed','yawspeed','xacc','yacc','zacc','xgyro','ygyro','zgyro', 'vx', 'vy', 'vz'],
    #     'state_cols': ['vx','vy','vz', 'relative_alt','RTL_ALT'],
    #     'constant_params': {},
    # },
    # 'A.RC_FS2': {
    #     'hang_thres': 420,
    #     'waypoints': 3,
    #     'wp_altitude': [30, 50],
    #     'params_to_fetch': ['RTL_ALT'],
    #     'params_to_fuzz': ['WPNAV_SPEED','WPNAV_RADIUS','WPNAV_SPEED_UP','WPNAV_SPEED_DN', 'WPNAV_ACCEL', 'WPNAV_ACCEL_Z', 'WPNAV_JERK', 'ANGLE_MAX'],
    #     'polygon_corners': polygon_corners['big'],
    #     'modes_vec': ['STABILIZE', 'AUTO', 'RTL'],
    #     'log_cols': ['MODE', 'lat', 'lon', 'relative_alt','roll','pitch','yaw','rollspeed', 'pitchspeed','yawspeed','xacc','yacc','zacc','xgyro','ygyro','zgyro', 'vx', 'vy', 'vz'],
    #     'state_cols': ['vx','vy','vz', 'relative_alt','RTL_ALT'],
    #     'constant_params': {},
    # },
    'A.SPORT1': {
        'hang_thres': 420,
        'waypoints': 3,
        'wp_altitude': [30, 50],
        'params_to_fetch': ['PILOT_SPEED_UP'],
        'params_to_fuzz': ['WPNAV_SPEED','WPNAV_RADIUS','WPNAV_SPEED_UP','WPNAV_SPEED_DN', 'WPNAV_ACCEL', 'WPNAV_ACCEL_Z', 'WPNAV_JERK', 'ANGLE_MAX'],
        'polygon_corners': polygon_corners['big'],
        'modes_vec': ['STABILIZE', 'AUTO', 'RTL'],
        'log_cols': ['MODE', 'lat', 'lon', 'relative_alt','roll','pitch','yaw','rollspeed', 'pitchspeed','yawspeed','xacc','yacc','zacc','xgyro','ygyro','zgyro', 'vx', 'vy', 'vz'],
        'state_cols': ['vx','vy','vz', 'relative_alt','RTL_ALT'],
        'constant_params': {},
    },
    # 'A.GUIDED1': {
    #     'hang_thres': 420,
    #     'waypoints': 3,
    #     'wp_altitude': [30, 50],
    #     'params_to_fetch': ['RTL_ALT'],
    #     'params_to_fuzz': ['WPNAV_SPEED','WPNAV_RADIUS','WPNAV_SPEED_UP','WPNAV_SPEED_DN', 'WPNAV_ACCEL', 'WPNAV_ACCEL_Z', 'WPNAV_JERK', 'ANGLE_MAX'],
    #     'polygon_corners': polygon_corners['big'],
    #     'modes_vec': ['STABILIZE', 'AUTO', 'RTL'],
    #     'log_cols': ['MODE', 'lat', 'lon', 'relative_alt','roll','pitch','yaw','rollspeed', 'pitchspeed','yawspeed','xacc','yacc','zacc','xgyro','ygyro','zgyro', 'vx', 'vy', 'vz'],
    #     'state_cols': ['vx','vy','vz', 'relative_alt','RTL_ALT'],
    #     'constant_params': {},
    # },
    'A.LOITER1': {
        'hang_thres': 420,
        'waypoints': 3,
        'wp_altitude': [30, 50],
        'params_to_fetch': [],
        'params_to_fuzz': ['WPNAV_SPEED','WPNAV_RADIUS','WPNAV_SPEED_UP','WPNAV_SPEED_DN', 'WPNAV_ACCEL', 'WPNAV_ACCEL_Z', 'WPNAV_JERK', 'ANGLE_MAX'],
        'polygon_corners': polygon_corners['big'],
        'modes_vec': ['STABILIZE', 'AUTO', 'RTL', 'LOITER'],
        'log_cols': ['MODE', 'lat', 'lon', 'alt','relative_alt', 'roll','pitch','yaw','rollspeed', 'pitchspeed','yawspeed','xacc','yacc','zacc','xgyro','ygyro','zgyro', 'vx', 'vy', 'vz'],
        'state_cols': ['vx','vy','vz', 'alt','RTL_ALT'],
        'constant_params': {},
    },
    # 'A.DRIFT1': {
    #     'hang_thres': 420,
    #     'waypoints': 3,
    #     'wp_altitude': [30, 50],
    #     'params_to_fetch': ['RTL_ALT'],
    #     'params_to_fuzz': ['WPNAV_SPEED','WPNAV_RADIUS','WPNAV_SPEED_UP','WPNAV_SPEED_DN', 'WPNAV_ACCEL', 'WPNAV_ACCEL_Z', 'WPNAV_JERK', 'ANGLE_MAX'],
    #     'polygon_corners': polygon_corners['big'],
    #     'modes_vec': ['STABILIZE', 'AUTO', 'RTL'],
    #     'log_cols': ['MODE', 'lat', 'lon', 'relative_alt','roll','pitch','yaw','rollspeed', 'pitchspeed','yawspeed','xacc','yacc','zacc','xgyro','ygyro','zgyro', 'vx', 'vy', 'vz'],
    #     'state_cols': ['vx','vy','vz', 'relative_alt','RTL_ALT'],
    #     'constant_params': {},
    # },
    # 'A.BRAKE1': {
    #     'hang_thres': 420,
    #     'waypoints': 3,
    #     'wp_altitude': [30, 50],
    #     'params_to_fetch': ['RTL_ALT'],
    #     'params_to_fuzz': ['WPNAV_SPEED','WPNAV_RADIUS','WPNAV_SPEED_UP','WPNAV_SPEED_DN', 'WPNAV_ACCEL', 'WPNAV_ACCEL_Z', 'WPNAV_JERK', 'ANGLE_MAX'],
    #     'polygon_corners': polygon_corners['big'],
    #     'modes_vec': ['STABILIZE', 'AUTO', 'RTL'],
    #     'log_cols': ['MODE', 'lat', 'lon', 'relative_alt','roll','pitch','yaw','rollspeed', 'pitchspeed','yawspeed','xacc','yacc','zacc','xgyro','ygyro','zgyro', 'vx', 'vy', 'vz'],
    #     'state_cols': ['vx','vy','vz', 'relative_alt','RTL_ALT'],
    #     'constant_params': {},
    # },
}

learn_vars = {
    'window_size_lstm': simulation_vars['set_wait_secs'], # will get the state vec from the last n different seconds
    'window_size_vio': simulation_vars['set_wait_secs'] * 5, # because roughly 5 records per second
}