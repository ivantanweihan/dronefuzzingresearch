# https://medium.com/the-data-journal/a-quick-trick-to-create-random-lat-long-coordinates-in-python-within-a-defined-polygon-e8997f05123a

import random
from pymavlink import mavutil
import random_points as random_points
import env_vars

# generate mission waypoints with randomized altitudes and speeds
def generate(curr_policy, sys, com):

    points = random_points.polygon_random_points(
        env_vars.simulation_vars[curr_policy]['polygon_corners'], 
        env_vars.simulation_vars[curr_policy]['waypoints']
    )

    mission_items = []
            
    for i in range(len(points)):
        x = points[i].x
        y = points[i].y
        # z = random.uniform(*env_vars.simulation_vars[curr_policy]['wp_altitude'])
        z = 50  # fix the way points

        current = 0
        frame = 3
        command = 16 #cmd_nav_wp

        if i == 0:
            current = 1
            frame = 0
            x = -35.3632622
            y = 149.1652375
        elif i == 1:
            command = 22 #take off
            x = -35.3632622
            y = 149.1652375
        elif i == len(points)-2:
            # fix the last way point
            x = -35.36238558169087
            y = 149.16693664137932
        elif i == len(mission_items)-1:
            # fixing the way points
            # x = -35.38238558169087
            # y = 149.16693664137932
            command = 21 #land
            z = 0

        wp = mavutil.mavlink.MAVLink_waypoint_message(
                sys, 
                com,
                i,    # seq
                frame,    # frame
                command,    # command
                current,    # current
                1,   # autocontinue
                0,  # param1,
                0,  # param2,
                0,  # param3
                0,  # param4
                x,  # x (latitude)
                y,  # y (longitude)
                z  # z (altitude)
            )
        mission_items.append(wp)

    return mission_items
