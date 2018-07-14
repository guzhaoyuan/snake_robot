// currently support read and write for 2 joint snake
#ifndef snakeHW_h
#define snakeHW_h

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include "dynamixel_sdk.h"

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
#define DXL_MINIMUM_POSITION_VALUE      1948                 // Dynamixel will rotate between this value
#define DXL_MIDDLE_POSITION_VALUE       2048
#define DXL_MAXIMUM_POSITION_VALUE      2148                 // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     10                  // Dynamixel moving status threshold

class Snake : public hardware_interface::RobotHW
{
public:
  Snake();
  ~Snake();
  bool configure();
  bool start();
  bool read() const; 
  void write(bool index);
  void stop();
  void cleanup();

private:
  dynamixel::PortHandler *portHandler;
  dynamixel::PacketHandler *packetHandler;
  // dynamixel::GroupSyncWrite groupSyncWrite;
  
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  
  double cmd[2];
  double pos[2];
  double vel[2];
  double eff[2];
};

#endif
