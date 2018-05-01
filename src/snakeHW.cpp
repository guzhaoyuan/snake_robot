#include <snakeHW.h>

Snake::Snake(){ 
 // connect and register the joint state interface
 hardware_interface::JointStateHandle state_handle_a("A", &pos[0], &vel[0], &eff[0]);
 jnt_state_interface.registerHandle(state_handle_a);

 hardware_interface::JointStateHandle state_handle_b("B", &pos[1], &vel[1], &eff[1]);
 jnt_state_interface.registerHandle(state_handle_b);

 registerInterface(&jnt_state_interface);

 // connect and register the joint position interface
 hardware_interface::JointHandle pos_handle_a(jnt_state_interface.getHandle("A"), &cmd[0]);
 jnt_pos_interface.registerHandle(pos_handle_a);

 hardware_interface::JointHandle pos_handle_b(jnt_state_interface.getHandle("B"), &cmd[1]);
 jnt_pos_interface.registerHandle(pos_handle_b);

 registerInterface(&jnt_pos_interface);
}

bool Snake::read(){
  return true;
}

void Snake::write(){
  
}