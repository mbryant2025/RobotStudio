#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32MultiArray.h"

#include <dlfcn.h>

#include <thread>


#define SERVO_COUNT 8
#define SCALE_FACTOR 4 // The data we send to the servo is 4*degrees

// Motor min and max values for each of the motors
std::vector<int> motor_mins = {0, 18, -10, 10, 6, 45, 6, 30};
std::vector<int> motor_maxs = {90, 60, 90, 50, 89, 86, 100, 65};

std::vector<int> motor_directions = {1, -1, 1, -1, -1, 1, -1, 1}; // Negate such that -1 will be moving the leg down and 1 will be moving the leg up

std::vector<int> motor_offsets = {-11, -11, -11, -11, 0, 0, 0, 0}; // For calibrating the sides relative to each other

std_msgs::Float32MultiArray command_positions;


// Send values -1 to 1
// -1 Is full leg down
// 1 Is full leg up
void servoCommandCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if (msg->data.size() == SERVO_COUNT) {
        command_positions.data = msg->data;
    }
    else {
        ROS_ERROR("Received servo command with wrong size: %d", (int)msg->data.size());
    }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "servo");
  ros::NodeHandle n;

  // Publishers for the servo positions and the servo command
  ros::Publisher servo_pub = n.advertise<std_msgs::Int32MultiArray>("servo_positions", 1000);
  ros::Subscriber servo_command_sub = n.subscribe("servo_commands", 1000, servoCommandCallback);

  std_msgs::Int32MultiArray positions;
  positions.data.resize(SERVO_COUNT);

  // Initialize the command_positions
  command_positions.data.resize(SERVO_COUNT);
  // Initialize command_positions to make it's legs like L
  command_positions.data[0] = -0.95;
  command_positions.data[2] = -0.95;
  command_positions.data[4] = -0.95;
  command_positions.data[6] = -0.95;

  command_positions.data[1] = 0.95;
  command_positions.data[3] = 0.95;
  command_positions.data[5] = 0.95;
  command_positions.data[7] = 0.95;


  ros::Rate loop_rate(10);

  // Load the shared library
  void* lib = dlopen("/opt/LX16A/lx16a.so", RTLD_NOW);
  if (!lib) {
    ROS_ERROR("Failed to load the shared library: %s", dlerror());
    return 1;
  }

  // int IO_init(char* filename);
  typedef int (*IO_init_t)(char*);
  IO_init_t IO_init = (IO_init_t)dlsym(lib, "IO_init");
  if (!IO_init) {
    ROS_ERROR("Failed to load the symbol: %s", dlerror());
    return 1;
  }

  // short posRead(char id);
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

  // void move(char id, short position, short time)
  typedef void (*move_t)(char, short, short);
  move_t move = (move_t)dlsym(lib, "move");
  if (!move) {
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

          // Set the servos to servo mode
          for(int i = 0; i < SERVO_COUNT ; i++) {
              setServoMode(i+1); // Servo IDs are 1-indexed
          }

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

  // We only want to get the servo positions every second (so every 10 iterations)
  int counter = 0;

  while (ros::ok())
  {

    if (counter == 0) {
        for(int i = 0; i < SERVO_COUNT ; i++) {
            positions.data[i] = posRead(i+1)/SCALE_FACTOR; // Servo IDs are 1-indexed
        }
        servo_pub.publish(positions);
    }
    counter = (counter + 1) % 10;

    // Command the servos, keeping in mind the min and max values
    for(int i = 0; i < SERVO_COUNT; i++){

      // command_positions will be in the range -1 to 1
      // We need to scale it between the min and max values, apply the offset as well

        int position_scaled = (motor_directions[i] * command_positions.data[i] + 1) * (motor_maxs[i] - motor_mins[i]) / 2 + motor_mins[i] + motor_offsets[i];
        if (position_scaled < motor_mins[i]) {
            position_scaled = motor_mins[i];
        }
        if (position_scaled > motor_maxs[i]) {
            position_scaled = motor_maxs[i];
        }

        move(i+1, position_scaled*SCALE_FACTOR, 150); // Servo IDs are 1-indexed
    }

    servo_pub.publish(positions);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}