#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"

#include <dlfcn.h>


#define SERVO_COUNT 8


int main(int argc, char **argv)
{
  ros::init(argc, argv, "servo");
  ros::NodeHandle n;

  // Create a publisher object
  ros::Publisher servo_pub = n.advertise<std_msgs::Int32MultiArray>("servo_positions", 1000);

  ros::Rate loop_rate(10);

  std_msgs::Int32MultiArray positions;
  positions.data.resize(SERVO_COUNT);

  void* lib = dlopen("/opt/LX16A/lx16a.so", RTLD_NOW);
  if (!lib) {
    ROS_ERROR("Failed to load the shared library: %s", dlerror());
    return 1;
  }

  // Exposes the following functions:
  // int IO_init(char* filename);
  // short posRead(char id);
  // void setServoMode(char id);

  typedef int (*IO_init_t)(char*);
  IO_init_t IO_init = (IO_init_t)dlsym(lib, "IO_init");
  if (!IO_init) {
    ROS_ERROR("Failed to load the symbol: %s", dlerror());
    return 1;
  }

  typedef short (*posRead_t)(char);
  posRead_t posRead = (posRead_t)dlsym(lib, "posRead");
  if (!posRead) {
    ROS_ERROR("Failed to load the symbol: %s", dlerror());
    return 1;
  }

  // void setServoMode(char id);
  typedef void (*setServoMode_t)(char);
  setServoMode_t setServoMode = (setServoMode_t)dlsym(lib, "setServoMode");
  if (!setServoMode) {
    ROS_ERROR("Failed to load the symbol: %s", dlerror());
    return 1;
  }

  bool init_successful = false;
  for (int i = 0; i <= 10; i++) {
      std::string device = "/dev/ttyUSB" + std::to_string(i);
      char* device_c_str = new char[device.length() + 1];
      std::strcpy(device_c_str, device.c_str());
      ROS_INFO("Trying to initialize the servo controller with device: %s", device_c_str);
      if (IO_init(device_c_str) == 1) {
          ROS_INFO("Successfully initialized the servo controller with device: %s", device_c_str);
          init_successful = true;
          delete[] device_c_str;
          break;
      }
      ROS_INFO("Failed to initialize the servo controller with device: %s", device_c_str);
      delete[] device_c_str;
  }

  if (!init_successful) {
      ROS_ERROR("Failed to initialize the servo controller with all tried devices");
      return 1;
  }




  while (ros::ok())
  {
    // Update servo positions here as needed
    for(int i = 0; i < SERVO_COUNT; i++) {
        positions.data[i] = i * 10; // Example position update
    }

    // Publish the message
    servo_pub.publish(positions);

    for(int i = 0; i < SERVO_COUNT; i++) {
        ROS_INFO("Servo %d position: %d", i, positions.data[i]);
    }

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}