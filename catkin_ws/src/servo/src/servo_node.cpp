#include "ros/ros.h"
#include "std_msgs/Int32.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "servo");
  ros::NodeHandle n;

  // Create a publisher object
  ros::Publisher servo_pub = n.advertise<std_msgs::Int32>("servo_position", 1000);

  ros::Rate loop_rate(10);

  int position = 0; // Initialize servo position

  while (ros::ok())
  {

    std_msgs::Int32 msg;
    msg.data = position;

    // Publish the message
    servo_pub.publish(msg);

    ROS_INFO("Servo position: %d", position);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}