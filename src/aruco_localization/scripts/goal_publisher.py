#!/usr/bin/env python3
import math
import rospy
from std_msgs.msg import Int32, Float32

def wrap_deg_180(deg):
    # folds any angle into [-180, +180]
    return ((deg + 180) % 360) - 180

def main():
    rospy.init_node('goal_publisher')

    id_pub    = rospy.Publisher('/goal_id',   Int32,   queue_size=1)
    theta_pub = rospy.Publisher('/goal_theta', Float32, queue_size=1)

    rospy.loginfo("Enter '<goal_id> <rotation_deg>' at any time (e.g. '2 45')")

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            line = input(">> ")
        except EOFError:
            break

        parts = line.strip().split()
        if len(parts) != 2:
            print("  invalid input, expected: <int> <float>")
            continue

        try:
            goal_id = int(parts[0])
            rot_deg = float(parts[1])
        except ValueError:
            print("  could not parse integers/floats")
            continue

        # wrap into [-180, 180]
        wrapped = wrap_deg_180(rot_deg)

        rot_rad = math.radians(wrapped)

        # publish
        id_pub.publish(goal_id)
        theta_pub.publish(rot_rad)
        rospy.loginfo("Published goal_id=%d, theta=%.3f rad (%.1f degree)", 
                      goal_id, rot_rad, wrapped)

        rate.sleep()

if __name__ == '__main__':
    main()
