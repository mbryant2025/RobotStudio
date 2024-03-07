#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray
import math

def publish_servo_commands():
    rospy.init_node("servo_command_publisher")
    servo_commands_pub = rospy.Publisher("servo_commands", Int32MultiArray, queue_size=1000)
    rate = rospy.Rate(10)  # Publish every 0.1 seconds

    while not rospy.is_shutdown():
        time = rospy.get_time()
        # Range from 50 to 100
        servo_command_values = [
            int(75 + 25 * math.cos(8*time)),
            int(75 + 25 * math.sin(8*time)) 
        ]

        servo_commands_msg = Int32MultiArray(data=servo_command_values)
        servo_commands_pub.publish(servo_commands_msg)

        rate.sleep()

if __name__ == "__main__":
    try:
        publish_servo_commands()
    except rospy.ROSInterruptException:
        pass
