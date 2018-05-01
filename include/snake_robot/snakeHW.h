
#ifndef snakeHW_h
#define snakeHW_h

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>


class Snake : public hardware_interface::RobotHW
{
public:
  Snake();
  bool configure();
  bool start();
  bool read(); 
  void write();
  void stop();
  void cleanup();

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  double cmd[2];
  double pos[2];
  double vel[2];
  double eff[2];
};

#endif
