// currently support read and write for 2 joint snake
#ifndef snakeHW_h
#define snakeHW_h

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <controller_manager/controller_manager.h>

#include <transmission_interface/transmission_interface.h>
#include <transmission_interface/simple_transmission.h>

#include "dynamixel_sdk.h"

#define pi                              3.1415926

// Protocol version
#define DXL1_ID                         1                   // Dynamixel#1 ID: 1
#define DXL2_ID                         2                   // Dynamixel#2 ID: 2
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel
#define BAUDRATE                        100000
#define DEVICENAME                      "/dev/ttyUSB0"

// Data Byte Length
#define LEN_MX_GOAL_POSITION            2
#define LEN_MX_PRESENT_POSITION         2

// Control table address
#define ADDR_MX_TORQUE_ENABLE           24                  // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION           30
#define ADDR_MX_PRESENT_POSITION        36

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      1535.0              // -45
#define DXL_MIDDLE_POSITION_VALUE       2047.0              // 0
#define DXL_MAXIMUM_POSITION_VALUE      2559.0              // 45
#define DXL_MOVING_STATUS_THRESHOLD     10                  // Dynamixel moving status threshold
#define DXL_REDUCE_RATE                 651.898658          //4096/3.1415926/2

class SnakeHW : public hardware_interface::RobotHW
{
public:
  SnakeHW(ros::NodeHandle nh_);
  ~SnakeHW();
  bool configure();
  bool start();
  void read(const ros::Time& time, const ros::Duration& period);
  void write(const ros::Time& time, const ros::Duration& period, bool index);
  void stop();
  void cleanup();
  double getFreq();

private:
  ros::NodeHandle nh;

  dynamixel::PortHandler *portHandler;
  dynamixel::PacketHandler *packetHandler;
  // dynamixel::GroupSyncWrite groupSyncWrite;
  
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  
  //Transmission interfaces
  transmission_interface::ActuatorToJointStateInterface act_to_jnt_state;
  transmission_interface::JointToActuatorPositionInterface jnt_to_act_pos;

  //Transmissions for the end two roll and pitch joints
  transmission_interface::SimpleTransmission sim_trans_joint1;
  transmission_interface::SimpleTransmission sim_trans_joint2;

  //Actuator and joint space variables
  transmission_interface::ActuatorData a_state_data[2]; // <- read from servo
  transmission_interface::JointData    j_state_data[2]; // transfer from a_state_data and input to controller
  transmission_interface::JointData    j_cmd_data[2];   // output from controller transfer to a_cmd_data
  transmission_interface::ActuatorData a_cmd_data[2];   // -> write to servo

  void TransJointToActuator();
  void TransActuatorToJoint();

  double freq;

  double a_curr_pos[2]; // Size 3: One per actuator
  double a_curr_vel[2];
  double a_curr_eff[2];
  double a_cmd_pos[2];

  double j_curr_pos[2]; // Size 3: One per joint
  double j_curr_vel[2];
  double j_curr_eff[2];
  double j_cmd_pos[2];


  double cmd[2];
  double pos[2];
  double vel[2];
  double eff[2];
};

#endif
