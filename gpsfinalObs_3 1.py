#!/usr/bin/env python

import rospy
import math
import time

from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import LaserScan

from collections import deque

# =========================================================
# TARGETS
# =========================================================

targets = [

    (47.47392, 19.05788),
    (47.47410, 19.05809),
    (47.47392, 19.05788)

]

targetid = 0

targetlat = targets[targetid][0]
targetlon = targets[targetid][1]

# =========================================================
# GLOBAL VARIABLES
# =========================================================

distance = 0
heading = 0

fix = False

northangle = 0

# =========================================================
# PARAMETERS
# =========================================================

targetstopdistance = 1.5

forwardspeed = 0.5
turnspeed = 0.6

devlimit = 30

# =========================================================
# OBSTACLE VARIABLES
# =========================================================

obstacle_danger = {

    'front': float('inf'),
    'left': float('inf'),
    'right': float('inf')

}

obstacle_history = deque(maxlen=5)

avoidance_mode = False
avoidance_start_time = 0

# =========================================================
# SAFETY THRESHOLDS
# =========================================================

SAFETY_CRITICAL = 0.5
SAFETY_WARNING = 0.8
SAFETY_CAUTION = 1.2

# =========================================================
# MAGNETOMETER
# =========================================================

def magcallback(imumsg):

    global northangle

    xcurrent = imumsg.magnetic_field.x
    ycurrent = imumsg.magnetic_field.y

    xnorth = 0.0
    ynorth = -0.5

    dot = xcurrent * xnorth + ycurrent * ynorth
    det = xnorth * ycurrent - ynorth * xcurrent

    angle = math.atan2(det, dot)

    northangle = angle * 180 / math.pi

# =========================================================
# GPS
# =========================================================

def gnsscallback(gnssmsg):

    global fix
    global distance
    global heading

    global targetlat
    global targetlon

    fix = True

    lat = gnssmsg.latitude
    lon = gnssmsg.longitude

    latdiff = targetlat - lat
    londiff = targetlon - lon

    latrad = lat * math.pi / 180
    lonrad = lon * math.pi / 180

    targetlatrad = targetlat * math.pi / 180
    targetlonrad = targetlon * math.pi / 180

    latdiffrad = latdiff * math.pi / 180
    londiffrad = londiff * math.pi / 180

    earthradius = 6371000

    a = (

        math.sin(latdiffrad / 2) ** 2

        +

        math.cos(latrad)
        * math.cos(targetlatrad)
        * math.sin(londiffrad / 2) ** 2

    )

    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    distance = earthradius * c

    y = math.sin(targetlonrad - lonrad) * math.cos(targetlatrad)

    x = (

        math.cos(latrad) * math.sin(targetlatrad)

        -

        math.sin(latrad)
        * math.cos(targetlatrad)
        * math.cos(targetlonrad - lonrad)

    )

    heading = math.atan2(y, x) * 180 / math.pi

# =========================================================
# LIDAR HELPERS
# =========================================================

def get_sector_min_distance(ranges, start_idx, end_idx):

    valid_distances = []

    for i in range(start_idx, end_idx):

        if i < 0 or i >= len(ranges):
            continue

        val = ranges[i]

        if (

            val is not None

            and not math.isinf(val)

            and not math.isnan(val)

            and val > 0.1

        ):

            valid_distances.append(val)

    if len(valid_distances) == 0:

        return float('inf')

    return min(valid_distances)

# =========================================================
# LIDAR CALLBACK
# =========================================================

def lasercallback(lasermsg):

    global obstacle_danger
    global obstacle_history

    try:

        num_ranges = len(lasermsg.ranges)

        center_idx = num_ranges // 2

        # wider sectors
        sector_width = int(num_ranges * 60 / 360)

        # left
        left_start = center_idx - sector_width
        left_end = center_idx

        # center
        center_start = center_idx - sector_width // 2
        center_end = center_idx + sector_width // 2

        # right
        right_start = center_idx
        right_end = center_idx + sector_width

        left_min = get_sector_min_distance(

            lasermsg.ranges,
            left_start,
            left_end

        )

        center_min = get_sector_min_distance(

            lasermsg.ranges,
            center_start,
            center_end

        )

        right_min = get_sector_min_distance(

            lasermsg.ranges,
            right_start,
            right_end

        )

        obstacle_danger = {

            'front': center_min,
            'left': left_min,
            'right': right_min

        }

        obstacle_present = center_min < SAFETY_CAUTION

        obstacle_history.append(obstacle_present)

    except:

        obstacle_danger = {

            'front': float('inf'),
            'left': float('inf'),
            'right': float('inf')

        }

        obstacle_history.append(False)

# =========================================================
# EVASION COMMAND
# =========================================================

def get_evasion_command():

    vel = Twist()

    front_dist = obstacle_danger['front']
    left_dist = obstacle_danger['left']
    right_dist = obstacle_danger['right']

    # =====================================================
    # CRITICAL
    # =====================================================

    if front_dist < SAFETY_CRITICAL:

        vel.linear.x = -0.1

        if left_dist > right_dist:

            vel.angular.z = 1.0

        else:

            vel.angular.z = -1.0

        print("CRITICAL OBSTACLE")

    # =====================================================
    # WARNING
    # =====================================================

    elif front_dist < SAFETY_WARNING:

        vel.linear.x = 0.05

        if left_dist > right_dist:

            vel.angular.z = 0.8

        else:

            vel.angular.z = -0.8

        print("WARNING OBSTACLE")

    # =====================================================
    # CAUTION
    # =====================================================

    elif front_dist < SAFETY_CAUTION:

        vel.linear.x = 0.2

        if left_dist > right_dist:

            vel.angular.z = 0.4

        else:

            vel.angular.z = -0.4

        print("CAUTION OBSTACLE")

    else:

        return None

    return vel

# =========================================================
# MAIN LOOP
# =========================================================

def listener():

    global targetlat
    global targetlon
    global targetid

    global avoidance_mode
    global avoidance_start_time

    rospy.init_node('gps_lidar_navigation')

    rospy.Subscriber(

        '/gnss',
        NavSatFix,
        gnsscallback

    )

    rospy.Subscriber(

        '/imu/mag',
        MagneticField,
        magcallback

    )

    rospy.Subscriber(

        '/scan',
        LaserScan,
        lasercallback

    )

    pub = rospy.Publisher(

        '/cmd_vel',
        Twist,
        queue_size=1

    )

    rate = rospy.Rate(10)

    reached_counter = 0

    while not rospy.is_shutdown():

        vel = Twist()

        if fix:

            print("===================================")

            print("TARGET:", targetid)

            print("Distance:", distance)

            print("Front:", obstacle_danger['front'])

            print("Left :", obstacle_danger['left'])

            print("Right:", obstacle_danger['right'])

            print("Heading:", heading)

            print("North:", northangle)

            # =================================================
            # TARGET REACHED
            # =================================================

            if distance < targetstopdistance:

                reached_counter += 1

            else:

                reached_counter = 0

            if reached_counter > 10:

                print("TARGET REACHED")

                vel.linear.x = 0
                vel.angular.z = 0

                pub.publish(vel)

                time.sleep(3)

                targetid += 1

                reached_counter = 0

                if targetid < len(targets):

                    targetlat = targets[targetid][0]
                    targetlon = targets[targetid][1]

                    print("NEXT TARGET:", targetid)

                else:

                    print("MISSION COMPLETE")

                    break

            # =================================================
            # ENTER AVOIDANCE MODE
            # =================================================

            if obstacle_danger['front'] < SAFETY_CAUTION:

                avoidance_mode = True

                avoidance_start_time = time.time()

            # =================================================
            # AVOIDANCE MODE
            # =================================================

            if avoidance_mode:

                print("AVOIDANCE MODE")

                evasion_cmd = get_evasion_command()

                if evasion_cmd is not None:

                    vel.linear.x = evasion_cmd.linear.x
                    vel.angular.z = evasion_cmd.angular.z

                elapsed = time.time() - avoidance_start_time

                # stay in avoidance longer
                if (

                    elapsed > 3.0

                    and obstacle_danger['front'] > 2.0

                ):

                    print("EXIT AVOIDANCE")

                    avoidance_mode = False

            # =================================================
            # NORMAL GPS NAVIGATION
            # =================================================

            else:

                current_speed = forwardspeed

                if (

                    obstacle_danger['left'] < SAFETY_CAUTION

                    or

                    obstacle_danger['right'] < SAFETY_CAUTION

                ):

                    current_speed = 0.3

                # heading check
                if (

                    northangle > heading - devlimit

                    and

                    northangle < heading + devlimit

                ):

                    vel.linear.x = current_speed

                turn = 0

                if northangle > heading:
                    turn = 1

                if northangle < heading:
                    turn = -1

                # wrap-around correction
                if northangle > 90 and heading < -90:
                    turn = -1

                if northangle < -90 and heading > 90:
                    turn = 1

                vel.angular.z = turn * turnspeed

        pub.publish(vel)

        rate.sleep()

# =========================================================
# START
# =========================================================

if __name__ == '__main__':

    listener()