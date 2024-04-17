#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray

def sit():
    rospy.init_node("servo_command_publisher")
    servo_commands_pub = rospy.Publisher("servo_commands", Float32MultiArray, queue_size=1000)

    if not rospy.is_shutdown():
        time = rospy.get_time()
         # Neutral positions (0.95 approximates 1 -- the max in one direction)
        servo_command_values = [-0.95, 0.95, -0.95, 0.95, -0.95, 0.95, -0.95, -0.95]
        servo_commands_msg = Float32MultiArray(data=servo_command_values)
        servo_commands_pub.publish(servo_commands_msg)


if __name__ == "__main__":
    try:
        sit()
    except rospy.ROSInterruptException:
        pass

