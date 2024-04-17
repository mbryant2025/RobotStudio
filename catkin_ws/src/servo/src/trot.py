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

def trot():
    

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

        # # Lift up the back left femur
        # servo_commands_msg = Float32MultiArray(data=servo_command_values)
        # servo_commands_pub.publish(servo_commands_msg)

        # rospy.sleep(0.4)

        # # Drop down the leg
        # servo_commands_msg = Float32MultiArray(data=servo_command_values)
        # servo_commands_pub.publish(servo_commands_msg)

        # rospy.sleep(0.4)

        # Now we will trot while the parameter is set to True
        rospy.loginfo("Trotting...")

        # What instructions to execute every timestep
        instructions_dict_1 = [
            {
                BACK_LEFT_LOWER_LEG: 0.9-0.05, # Lift up the lower legs
                FRONT_RIGHT_LOWER_LEG: 0.9,
                BACK_LEFT_FEMUR: 0.7-0.2, # Lift up the femurs
                FRONT_RIGHT_FEMUR: 0.7
            },

            {
                BACK_LEFT_FEMUR: -0.7-0.2, # Throw forward the femurs
                FRONT_RIGHT_FEMUR: -0.7
            },

            {
                BACK_LEFT_LOWER_LEG: -0.55-0.2, # Drop down the lower legs
                FRONT_RIGHT_LOWER_LEG: -0.55
            },

            {
                BACK_LEFT_FEMUR: 0.15-0.1, # Bring back to starting position
                BACK_LEFT_LOWER_LEG: -0.95-0.03,
            },

            {},

            {
                
                FRONT_RIGHT_FEMUR: 0.15, # Bring back to starting position pt 2
                FRONT_RIGHT_LOWER_LEG: -0.95
            }
        ]

        # Create the second instructions as a clone of the first, but we replace the front rights with from lefts, and such
        segment_map = {
            BACK_LEFT_LOWER_LEG: BACK_RIGHT_LOWER_LEG,
            FRONT_RIGHT_LOWER_LEG: FRONT_LEFT_LOWER_LEG,
            BACK_LEFT_FEMUR: BACK_RIGHT_FEMUR,
            FRONT_RIGHT_FEMUR: FRONT_LEFT_FEMUR
        }

        instructions_dict_2 = []

        for instruction in instructions_dict_1:
            new_instruction = {}
            for servo, value in instruction.items():
                new_instruction[segment_map[servo]] = value
            instructions_dict_2.append(new_instruction)
           

        instruction_offset = 3 # The delay in index between the two instructions


        while not rospy.is_shutdown():

            # Account for offset between the two instructions
            num_instructions = len(instructions_dict_1)

            for i in range(num_instructions):
                instruction_1 = instructions_dict_1[i]
                instruction_2 = instructions_dict_2[(i + instruction_offset) % num_instructions]

                for servo, value in instruction_1.items():
                    servo_command_values[servo] = value

                for servo, value in instruction_2.items():
                    servo_command_values[servo] = value

                servo_commands_msg = Float32MultiArray(data=servo_command_values)
                servo_commands_pub.publish(servo_commands_msg)

                # rospy.sleep(0.08)
                rospy.sleep(0.1)

                if rospy.is_shutdown():
                    break



if __name__ == "__main__":
    try:
        trot()
    except rospy.ROSInterruptException:
        pass

