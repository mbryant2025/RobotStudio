#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
import math
import signal


BACK_LEFT_FEMUR = 0
BACK_LEFT_LOWER_LEG = 1

FRONT_LEFT_FEMUR = 2
FRONT_LEFT_LOWER_LEG = 3

BACK_RIGHT_FEMUR = 4
BACK_RIGHT_LOWER_LEG = 5

FRONT_RIGHT_FEMUR = 6
FRONT_RIGHT_LOWER_LEG = 7

servo_command_values = [-0.95, 0.95, -0.95, 0.95, -0.95, 0.95, -0.95, 0.95]


rospy.init_node("servo_command_publisher")
servo_commands_pub = rospy.Publisher("servo_commands", Float32MultiArray, queue_size=1000)


def on_exit(sig, frame):
    if not rospy.is_shutdown():
        # Neutral positions (0.95 approximates 1 -- the max in one direction)
        servo_command_values = [-0.95, 0.95, -0.95, 0.95, -0.95, 0.95, -0.95, 0.95]
        servo_commands_msg = Float32MultiArray(data=servo_command_values)
        servo_commands_pub.publish(servo_commands_msg)
        rospy.signal_shutdown("Keyboard interrupt")
        rospy.loginfo("Sitting down...")
        exit(0)


def kneel():
    if not rospy.is_shutdown():
        servo_command_values = [-0.95, 0.95, -0.95, 0.95, -0.95, 0.95, -0.95, 0.95]
        servo_commands_msg = Float32MultiArray(data=servo_command_values)
        servo_commands_pub.publish(servo_commands_msg)

def back():
    if not rospy.is_shutdown():
        servo_command_values = [-0.95, -0.95, -0.95, -0.95, -0.95, -0.95, -0.95, -0.95]
        servo_commands_msg = Float32MultiArray(data=servo_command_values)
        servo_commands_pub.publish(servo_commands_msg)

def down():
    kneel()
    rospy.sleep(0.2)
    if not rospy.is_shutdown():
        servo_command_values = [0.95, 0.95, 0.95, 0.95, 0.95, 0.95, 0.95, 0.95]
        servo_commands_msg = Float32MultiArray(data=servo_command_values)
        servo_commands_pub.publish(servo_commands_msg)

def stand():
    if not rospy.is_shutdown():
        servo_command_values = [0.15, -0.95, 0.15, -0.95, 0.15, -0.95, 0.15, -0.95]
        servo_commands_msg = Float32MultiArray(data=servo_command_values)
        servo_commands_pub.publish(servo_commands_msg)

def around(rotations):
    stand()
    if not rospy.is_shutdown():
        # Drop each femur one by one
        for _ in range(rotations):

            servo_command_values = [-0.95, 0.95, -0.95, 0.95, -0.95, 0.95, -0.95, 0.95]

            servo_command_values[FRONT_RIGHT_FEMUR] = 0.95 # Move robot down

            servo_commands_msg = Float32MultiArray(data=servo_command_values)
            servo_commands_pub.publish(servo_commands_msg)

            rospy.sleep(0.3)

            servo_command_values[FRONT_LEFT_FEMUR] = 0.95

            servo_commands_msg = Float32MultiArray(data=servo_command_values)
            servo_commands_pub.publish(servo_commands_msg)

            rospy.sleep(0.3)


            servo_command_values[BACK_LEFT_FEMUR] = 0.95

            servo_commands_msg = Float32MultiArray(data=servo_command_values)
            servo_commands_pub.publish(servo_commands_msg)

            rospy.sleep(0.3)

            servo_command_values[FRONT_RIGHT_FEMUR] = -0.95 # Restore
            servo_command_values[BACK_RIGHT_FEMUR] = 0.95


            servo_commands_msg = Float32MultiArray(data=servo_command_values)
            servo_commands_pub.publish(servo_commands_msg)

            rospy.sleep(0.3)

            servo_command_values[FRONT_LEFT_FEMUR] = -0.95 # Restore

            servo_commands_msg = Float32MultiArray(data=servo_command_values)
            servo_commands_pub.publish(servo_commands_msg)


            rospy.sleep(0.3)

            servo_command_values[BACK_LEFT_FEMUR] = -0.95 # Restore

            servo_commands_msg = Float32MultiArray(data=servo_command_values)
            servo_commands_pub.publish(servo_commands_msg)


            rospy.sleep(0.3)

            servo_command_values[BACK_RIGHT_FEMUR] = -0.95 # Restore

            servo_commands_msg = Float32MultiArray(data=servo_command_values)
            servo_commands_pub.publish(servo_commands_msg)



def punch(num_iterations):

    if not rospy.is_shutdown():

        servo_command_values = [-0.95, -0.95, -0.95, -0.95, -0.95, -0.95, -0.95, -0.95]

        servo_command_values[FRONT_RIGHT_FEMUR] = -0.35
        servo_command_values[FRONT_LEFT_FEMUR] = -0.35

        # servo_commands_msg = Float32MultiArray(data=servo_command_values)
        # servo_commands_pub.publish(servo_commands_msg)

        # rospy.sleep(0.8)


        for _ in range(num_iterations):

            # Move the lower legs up and down
            servo_command_values[FRONT_LEFT_LOWER_LEG] = -1
            servo_command_values[FRONT_RIGHT_LOWER_LEG] = 1

            servo_commands_msg = Float32MultiArray(data=servo_command_values)
            servo_commands_pub.publish(servo_commands_msg)

            rospy.sleep(0.5)

            servo_command_values[FRONT_LEFT_LOWER_LEG] = 1
            servo_command_values[FRONT_RIGHT_LOWER_LEG] = -1

            servo_commands_msg = Float32MultiArray(data=servo_command_values)
            servo_commands_pub.publish(servo_commands_msg)

            rospy.sleep(0.5)

            
def cry(num_iterations):

    # Move front right femur up and down
    for _ in range(num_iterations):

        servo_command_values = [-0.95, 0.95, -0.95, 0.95, -0.95, 0.95, -0.95, 0.95]

        servo_command_values[FRONT_RIGHT_FEMUR] = 0.95

        servo_commands_msg = Float32MultiArray(data=servo_command_values)
        servo_commands_pub.publish(servo_commands_msg)

        rospy.sleep(0.3)

        servo_command_values[FRONT_RIGHT_FEMUR] = 0

        servo_commands_msg = Float32MultiArray(data=servo_command_values)
        servo_commands_pub.publish(servo_commands_msg)

        rospy.sleep(0.3)

    rospy.sleep(0.6)

    servo_command_values = [-0.95, 0.95, -0.95, 0.95, -0.95, 0.95, -0.95, 0.95]

    servo_commands_msg = Float32MultiArray(data=servo_command_values)
    servo_commands_pub.publish(servo_commands_msg)


            
def wave(num_iterations):

    # Moves head and butt up and down, out of phase

    for _ in range(num_iterations):
            
        servo_command_values = [-0.95, 0.95, -0.95, 0.95, -0.95, 0.95, -0.95, 0.95]

        servo_command_values[FRONT_RIGHT_FEMUR] = 0.5
        servo_command_values[FRONT_LEFT_FEMUR] = 0.5

        servo_command_values[BACK_RIGHT_FEMUR] = -0.5
        servo_command_values[BACK_LEFT_FEMUR] = -0.5

        servo_commands_msg = Float32MultiArray(data=servo_command_values)
        servo_commands_pub.publish(servo_commands_msg)

        rospy.sleep(0.75)

        servo_command_values[FRONT_RIGHT_FEMUR] = -0.5
        servo_command_values[FRONT_LEFT_FEMUR] = -0.5

        servo_command_values[BACK_RIGHT_FEMUR] = 0.5
        servo_command_values[BACK_LEFT_FEMUR] = 0.5

        servo_commands_msg = Float32MultiArray(data=servo_command_values)
        servo_commands_pub.publish(servo_commands_msg)

        rospy.sleep(0.75)

    rospy.sleep(0.6)

    servo_command_values = [-0.95, 0.95, -0.95, 0.95, -0.95, 0.95, -0.95, 0.95]

    servo_commands_msg = Float32MultiArray(data=servo_command_values)
    servo_commands_pub.publish(servo_commands_msg)


def surge(num_iterations):

    # Moves robot in and out

    servo_command_values = [-0.95, 0.95, -0.95, 0.95, -0.95, 0.95, -0.95, 0.95]

    servo_commands_msg = Float32MultiArray(data=servo_command_values)
    servo_commands_pub.publish(servo_commands_msg)

    rospy.sleep(0.75)

    for _ in range(num_iterations):

        servo_command_values[FRONT_RIGHT_FEMUR] = 0
        servo_command_values[FRONT_LEFT_FEMUR] = 0
        servo_command_values[BACK_RIGHT_FEMUR] = 0
        servo_command_values[BACK_LEFT_FEMUR] = 0

        servo_commands_msg = Float32MultiArray(data=servo_command_values)
        servo_commands_pub.publish(servo_commands_msg)

        rospy.sleep(0.6)

        servo_command_values[FRONT_RIGHT_FEMUR] = -0.95
        servo_command_values[FRONT_LEFT_FEMUR] = -0.95
        servo_command_values[BACK_RIGHT_FEMUR] = -0.95
        servo_command_values[BACK_LEFT_FEMUR] = -0.95

        servo_commands_msg = Float32MultiArray(data=servo_command_values)
        servo_commands_pub.publish(servo_commands_msg)

        rospy.sleep(0.6)

    servo_command_values = [-0.95, 0.95, -0.95, 0.95, -0.95, 0.95, -0.95, 0.95]

    servo_commands_msg = Float32MultiArray(data=servo_command_values)
    servo_commands_pub.publish(servo_commands_msg)
    
            
    



def dance():
    

    if not rospy.is_shutdown():

        signal.signal(signal.SIGINT, on_exit)


        # Neutral positions (0.95 approximates 1 -- the max in one direction)
        servo_command_values = [-0.95, 0.95, -0.95, 0.95, -0.95, 0.95, -0.95, 0.95]
        servo_commands_msg = Float32MultiArray(data=servo_command_values)
        servo_commands_pub.publish(servo_commands_msg)

        rospy.sleep(0.8)

        # Lift up the front femurs
        servo_command_values[FRONT_RIGHT_FEMUR] = 0.15
        servo_command_values[FRONT_LEFT_FEMUR] = 0.15

        servo_commands_msg = Float32MultiArray(data=servo_command_values)
        servo_commands_pub.publish(servo_commands_msg)

        rospy.sleep(0.4)

        # Drop down the legs
        servo_command_values[FRONT_RIGHT_LOWER_LEG] = -0.95
        servo_command_values[FRONT_LEFT_LOWER_LEG] = -0.95

        servo_commands_msg = Float32MultiArray(data=servo_command_values)
        servo_commands_pub.publish(servo_commands_msg)

        rospy.sleep(0.4)


        # Lift up the back femurs
        servo_command_values[BACK_RIGHT_FEMUR] = 0.15
        servo_command_values[BACK_LEFT_FEMUR] = 0.15
        servo_commands_msg = Float32MultiArray(data=servo_command_values)
        servo_commands_pub.publish(servo_commands_msg)

        rospy.sleep(0.4)

        # Drop down the legs
        servo_command_values[BACK_RIGHT_LOWER_LEG] = -0.95
        servo_command_values[BACK_LEFT_LOWER_LEG] = -0.95

        servo_commands_msg = Float32MultiArray(data=servo_command_values)
        servo_commands_pub.publish(servo_commands_msg)

        rospy.sleep(0.4)

        # Now we will trot while the parameter is set to True
        rospy.loginfo("Dancing...")

        if not rospy.is_shutdown():
            back()

            rospy.sleep(1)

            punch(10)

            rospy.sleep(0.5)

            kneel()

            rospy.sleep(2)

            stand()

            rospy.sleep(2)

            down()

            rospy.sleep(1.3)

            around(1)

            rospy.sleep(2.6)

            cry(6)

            rospy.sleep(0.3)

            surge(5)

            rospy.sleep(0.3)

            punch(7)
           
            back()

            kneel()

            rospy.sleep(1.0)

            stand()

            rospy.sleep(2)

            down()

            rospy.sleep(1.3)

            around(1)

            rospy.sleep(2.6)

            cry(3)

            rospy.sleep(0.5)

            back()

            rospy.sleep(1)

            punch(20)




            rospy.sleep(20)



if __name__ == "__main__":
    try:
        dance()
    except rospy.ROSInterruptException:
        pass

