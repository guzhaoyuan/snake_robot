// robotHW that communicate with real hardware
// the file is invoked by main.cpp

#include <snakeHW.h>
#include <ros/console.h>
#include <algorithm>

SnakeHW::SnakeHW(ros::NodeHandle nh_)
        :nh(nh_)
        ,sim_trans_joint1(DXL_REDUCE_RATE, -pi)
        ,sim_trans_joint2(DXL_REDUCE_RATE, -pi)
{ 
  // connect and register the joint state interface
  // claim joint1 as a joint resource which relate with pos[0]
  // and controller will control joint1 according to snake_controllers.yaml
  hardware_interface::JointStateHandle state_handle1("joint1", &j_curr_pos[0], &j_curr_vel[0], &j_curr_eff[0]);
  jnt_state_interface.registerHandle(state_handle1);

  hardware_interface::JointStateHandle state_handle2("joint2", &j_curr_pos[1], &j_curr_vel[1], &j_curr_eff[1]);
  jnt_state_interface.registerHandle(state_handle2);

  registerInterface(&jnt_state_interface);

  // connect and register the joint position interface
  hardware_interface::JointHandle pos_handle1(jnt_state_interface.getHandle("joint1"), &j_cmd_pos[0]);
  jnt_pos_interface.registerHandle(pos_handle1);

  hardware_interface::JointHandle pos_handle2(jnt_state_interface.getHandle("joint2"), &j_cmd_pos[1]);
  jnt_pos_interface.registerHandle(pos_handle2);

  registerInterface(&jnt_pos_interface);


  // register transmission
  a_state_data[0].position.push_back(&a_curr_pos[0]);
  a_state_data[0].velocity.push_back(&a_curr_vel[0]);
  a_state_data[0].effort.push_back(&a_curr_eff[0]);

  j_state_data[0].position.push_back(&j_curr_pos[0]);
  j_state_data[0].velocity.push_back(&j_curr_vel[0]);
  j_state_data[0].effort.push_back(&j_curr_eff[0]);

  a_cmd_data[0].position.push_back(&a_cmd_pos[0]);
  j_cmd_data[0].position.push_back(&j_cmd_pos[0]);

  a_state_data[1].position.push_back(&a_curr_pos[1]);
  a_state_data[1].velocity.push_back(&a_curr_vel[1]);
  a_state_data[1].effort.push_back(&a_curr_eff[1]);

  j_state_data[1].position.push_back(&j_curr_pos[1]);
  j_state_data[1].velocity.push_back(&j_curr_vel[1]);
  j_state_data[1].effort.push_back(&j_curr_eff[1]);

  a_cmd_data[1].position.push_back(&a_cmd_pos[1]);
  j_cmd_data[1].position.push_back(&j_cmd_pos[1]);

  act_to_jnt_state.registerHandle(transmission_interface::ActuatorToJointStateHandle("trans1",
                                                             &sim_trans_joint1,
                                                             a_state_data[0],
                                                             j_state_data[0]));

  act_to_jnt_state.registerHandle(transmission_interface::ActuatorToJointStateHandle("trans2",
                                                             &sim_trans_joint2,
                                                             a_state_data[1],
                                                             j_state_data[1]));

  jnt_to_act_pos.registerHandle(transmission_interface::JointToActuatorPositionHandle("trans1",
                                                              &sim_trans_joint1,
                                                              a_cmd_data[0],
                                                              j_cmd_data[0]));

  jnt_to_act_pos.registerHandle(transmission_interface::JointToActuatorPositionHandle("trans2",
                                                              &sim_trans_joint2,
                                                              a_cmd_data[1],
                                                              j_cmd_data[1]));
// config dynamixel
  configure();
}

SnakeHW::~SnakeHW(){
	int dxl_comm_result = COMM_TX_FAIL;             // Communication result
	uint8_t dxl_error = 0;
  // Disable Dynamixel#1 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }

  // Disable Dynamixel#2 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }

	// Close port
  portHandler->closePort();
  std::cout<< "Leaving torque disabled and port closed\n";
}

void SnakeHW::read(const ros::Time& time, const ros::Duration& period){
	int dxl_comm_result = COMM_TX_FAIL;             // Communication result
	uint8_t dxl_error = 0;                          // Dynamixel error
	uint16_t dxl1_present_position = 0, dxl2_present_position = 0;                        // Present position

	// Read Dynamixel#1 present position
  dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL1_ID, ADDR_MX_PRESENT_POSITION, &dxl1_present_position, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }

  // Read Dynamixel#2 present position
  dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL2_ID, ADDR_MX_PRESENT_POSITION, &dxl2_present_position, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }

  a_curr_pos[0] = (double)dxl1_present_position;
  a_curr_pos[1] = (double)dxl2_present_position;
  
  ROS_INFO("[ID:%03d] a_curr_pos:%03f\t[ID:%03d] a_curr_pos:%03f.", DXL1_ID, a_curr_pos[0], DXL2_ID, a_curr_pos[1]);
  
  act_to_jnt_state.propagate();
}

void SnakeHW::write(const ros::Time& time, const ros::Duration& period){
  jnt_to_act_pos.propagate();
// use a_cmd_pos
  ROS_INFO("cmd1:%f, cmd2:%f.", a_cmd_pos[0], a_cmd_pos[1]);

  assert(a_cmd_pos[0]>DXL_MINIMUM_POSITION_VALUE && a_cmd_pos[0]<DXL_MAXIMUM_POSITION_VALUE);
  assert(a_cmd_pos[1]>DXL_MINIMUM_POSITION_VALUE && a_cmd_pos[1]<DXL_MAXIMUM_POSITION_VALUE);

  uint16_t command1 = a_cmd_pos[0];
  uint16_t command2 = a_cmd_pos[1];

	int dxl_comm_result = COMM_TX_FAIL;             // Communication result
	uint8_t dxl_error = 0;                          // Dynamixel error
	uint8_t param_goal_position[2];
	bool dxl_addparam_result = false;               // addParam result
	dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION);

  // Allocate goal position value into byte array
  param_goal_position[0] = DXL_LOBYTE(command1);
  param_goal_position[1] = DXL_HIBYTE(command1);

  // Add Dynamixel#1 goal position value to the Syncwrite storage
  dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_goal_position);
  if (dxl_addparam_result != true)
  {
    fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL1_ID);
    return;
  }

  param_goal_position[0] = DXL_LOBYTE(command2);
  param_goal_position[1] = DXL_HIBYTE(command2);
  // Add Dynamixel#2 goal position value to the Syncwrite parameter storage
  dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, param_goal_position);
  if (dxl_addparam_result != true)
  {
    fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL2_ID);
    return;
  }

  // Syncwrite goal position
  dxl_comm_result = groupSyncWrite.txPacket();
  if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

  // Clear syncwrite parameter storage
  groupSyncWrite.clearParam();
}

bool SnakeHW::configure(){

  freq = 100.0;
  a_curr_pos[0] = DXL_MIDDLE_POSITION_VALUE; a_curr_pos[1] = DXL_MIDDLE_POSITION_VALUE;
  j_cmd_pos[0] = 0; j_cmd_pos[1] = 0;

	portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
	packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
	
	int dxl_comm_result = COMM_TX_FAIL;             // Communication result

  uint8_t dxl_error = 0;                          // Dynamixel error
  uint16_t dxl_model_number;                      // Dynamixel model number

	// Open port
  if (portHandler->openPort())
  {
    printf("Succeeded to open the port!\n");
  }
  else
  {
    printf("Failed to open the port!\n");
    return 0;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    printf("Succeeded to change the baudrate!\n");
  }
  else
  {
    printf("Failed to change the baudrate!\n");
    return 0;
  }

  // Try to ping the Dynamixel
  // Get Dynamixel model number
  for(int i = 1; i < 8; i++){
    dxl_comm_result = packetHandler->ping(portHandler, i, &dxl_model_number, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      printf("[ID:%03d] %s\n", i, packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      printf("[ID:%03d] %s\n", i, packetHandler->getRxPacketError(dxl_error));
    }
    else
    {
      printf("[ID:%03d] ping Succeeded. Dynamixel model number : %d\n", i, dxl_model_number);
    }
  }


  // Enable Dynamixel#1 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }
  else
  {
    printf("Dynamixel#%d has been successfully connected \n", DXL1_ID);
  }

  // Enable Dynamixel#2 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }
  else
  {
    printf("Dynamixel#%d has been successfully connected \n", DXL2_ID);
  }

  writeHome();

  if(start())
  {
    ROS_INFO("snakeHW initialized successfully");
  }
  else
    return 0;
}

bool SnakeHW::start(){
  ROS_INFO("STARTING!!!!");
	return true;
}

void SnakeHW::stop(){

}
void SnakeHW::cleanup(){

}

double SnakeHW::getFreq(){
  return freq;
}

void SnakeHW::writeHome(){
  ROS_INFO("Home the Snake");

  uint16_t command1 = DXL_MIDDLE_POSITION_VALUE;
  uint16_t command2 = DXL_MIDDLE_POSITION_VALUE;
  ROS_INFO("command1:%d ; command2:%d", command1, command2);

  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  uint8_t dxl_error = 0;                          // Dynamixel error
  uint8_t param_goal_position[2];
  bool dxl_addparam_result = false;               // addParam result
  dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION);

  // Allocate goal position value into byte array
  param_goal_position[0] = DXL_LOBYTE(command1);
  param_goal_position[1] = DXL_HIBYTE(command1);

  // Add Dynamixel#1 goal position value to the Syncwrite storage
  dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_goal_position);
  if (dxl_addparam_result != true)
  {
    fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL1_ID);
    return;
  }

  param_goal_position[0] = DXL_LOBYTE(command2);
  param_goal_position[1] = DXL_HIBYTE(command2);
  // Add Dynamixel#2 goal position value to the Syncwrite parameter storage
  dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, param_goal_position);
  if (dxl_addparam_result != true)
  {
    fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL2_ID);
    return;
  }

  // Syncwrite goal position
  dxl_comm_result = groupSyncWrite.txPacket();
  if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

  // Clear syncwrite parameter storage
  groupSyncWrite.clearParam();
}