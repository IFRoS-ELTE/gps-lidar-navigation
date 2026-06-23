#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix, MagneticField, LaserScan

# ===== PARAMETERS =====
targets = [
    (47.4739296, 19.0579776),  # target 0: gate
    (47.4740669, 19.0579415),  # target 1: entrance
    (47.4740227, 19.0577421)   # target 2: garden center
]
targetid = 0
distance, heading, fix, northangle = 0, 0, False, 0

obstacle_danger = {'front': float('inf'), 'left': float('inf'), 'right': float('inf')}
SAFETY_CRITICAL = 0.55  # Minimum front distance (emergency)
SAFETY_SIDE     = 0.50  # Minimum side distance
STOP_DISTANCE   = 2.0   # Distance to consider the target reached
FORWARD_SPEED   = 0.4
TURN_SPEED      = 0.5
DEV_LIMIT       = 15    # Angular tolerance in degrees

# ===== CALLBACKS =====
def magcallback(msg):
    global northangle
    # FIX: eje correcto para atan2 y referencia norte
    x = msg.magnetic_field.x
    y = msg.magnetic_field.y
    northangle = math.atan2(y, x) * 180 / math.pi

def gnsscallback(msg):
    global fix, distance, heading
    if msg.status.status == 0:
        fix = True
        lat, lon = msg.latitude, msg.longitude
        t_lat, t_lon = targets[targetid]

        latrad      = lat   * math.pi / 180
        lonrad      = lon   * math.pi / 180
        t_latrad    = t_lat * math.pi / 180
        t_lonrad    = t_lon * math.pi / 180
        latdiffrad  = (t_lat - lat) * math.pi / 180
        londiffrad  = (t_lon - lon) * math.pi / 180

        # Haversine
        a = (math.sin(latdiffrad / 2) ** 2 +
             math.cos(latrad) * math.cos(t_latrad) * math.sin(londiffrad / 2) ** 2)
        distance = 6371000 * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        # Bearing
        y = math.sin(t_lonrad - lonrad) * math.cos(t_latrad)
        x = (math.cos(latrad) * math.sin(t_latrad) -
             math.sin(latrad) * math.cos(t_latrad) * math.cos(londiffrad))
        heading = math.atan2(y, x) * 180 / math.pi

def lasercallback(msg):
    global obstacle_danger
    ranges = msg.ranges
    num = len(ranges)

    # Divide into 3 sectors centered at the front
    # Adjust these indices according to the physical orientation of the LiDAR
    front_half = num // 6   # ±30° at the front
    side_span  = num // 4   # lateral sector ~90°
    center     = num // 2

    def safe_min(sector):
        valid = [r for r in sector if msg.range_min < r < msg.range_max]
        return min(valid) if valid else float('inf')  # FIX: avoid min() from empty list

    obstacle_danger['front'] = safe_min(ranges[center - front_half : center + front_half])
    obstacle_danger['left']  = safe_min(ranges[center + front_half : center + front_half + side_span])
    obstacle_danger['right'] = safe_min(ranges[center - front_half - side_span : center - front_half])

# ===== CONTROL LOGIC =====
def angle_diff(a, b):
    """angular difference normalized on [-180, 180]."""
    return (a - b + 180) % 360 - 180

def get_control_command():
    vel = Twist()
    front = obstacle_danger['front']
    left  = obstacle_danger['left']
    right = obstacle_danger['right']

    # Priority 1: Avoid obstacles
    if front < SAFETY_CRITICAL:
        vel.linear.x  = -0.15  # go backwards
        vel.angular.z =  0.4 if right > left else -0.4  # Girar hacia el lado más libre
        rospy.logwarn("OBSTACLE FRONT | Reversing | L:%.2f F:%.2f R:%.2f" % (left, front, right))
        return vel, True

    if left < SAFETY_SIDE:
        vel.linear.x  =  0.15
        vel.angular.z = -0.4   # Avoid on the right
        rospy.logwarn("OBSTACLE LEFT  | Dodging right")
        return vel, True

    if right < SAFETY_SIDE:
        vel.linear.x  =  0.15
        vel.angular.z =  0.4   # Avoid on the left
        rospy.logwarn("OBSTACLE RIGHT | Dodging left")
        return vel, True

    # Priority 2: GPS Navegation 
    error = angle_diff(heading, northangle)
    if abs(error) < DEV_LIMIT:
        vel.linear.x  = FORWARD_SPEED
        vel.angular.z = 0
    else:
        vel.linear.x  = 0.1
        vel.angular.z = TURN_SPEED if error > 0 else -TURN_SPEED

    rospy.loginfo("NAV | Target:%d Dist:%.2fm Error:%.1f°" % (targetid, distance, error))
    return vel, False

# ===== MAIN NODE =====
def listener():
    global targetid

    rospy.init_node('autonomous_robot', anonymous=True)
    rospy.Subscriber('/gnss',	    NavSatFix,    gnsscallback)
    rospy.Subscriber('/imu/mag', MagneticField, magcallback)
    rospy.Subscriber('/scan',    LaserScan,     lasercallback)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        vel = Twist()  # stoped by default

        if fix:
            if distance > STOP_DISTANCE:
                vel, _ = get_control_command()
            else:
                # Target alcanzado
                rospy.loginfo("Reached target %d! Waiting 5s..." % targetid)
                pub.publish(vel)   # Stopped before waiting
                rospy.sleep(5)

                targetid += 1
                if targetid < len(targets):
                    rospy.loginfo("Moving to target %d" % targetid)
                else:
                    rospy.loginfo("Mission complete!")
                    break

        pub.publish(vel)
        rate.sleep()

if __name__ == '__main__':
    listener()
