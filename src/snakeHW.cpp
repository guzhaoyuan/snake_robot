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

 configure();
}

Snake::~Snake(){
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

bool Snake::read() const{
	int dxl_comm_result = COMM_TX_FAIL;             // Communication result
	uint8_t dxl_error = 0;                          // Dynamixel error
	uint16_t dxl1_present_position = 0, dxl2_present_position = 0;                        // Present position

	// Read Dynamixel#1 present position
  dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL1_ID, ADDR_MX_PRESENT_POSITION, &dxl1_present_position, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    return false;
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    return false;
  }

  // Read Dynamixel#2 present position
  dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL2_ID, ADDR_MX_PRESENT_POSITION, &dxl2_present_position, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    return false;
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    return false;
  }

  printf("[ID:%03d] PresPos:%03d\t[ID:%03d] PresPos:%03d\n", DXL1_ID, dxl1_present_position, DXL2_ID, dxl2_present_position);
  return true;
}

void Snake::write(bool index){
	int dxl_comm_result = COMM_TX_FAIL;             // Communication result
	uint8_t dxl_error = 0;                          // Dynamixel error
	uint8_t param_goal_position[2];
	bool dxl_addparam_result = false;               // addParam result
	dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION);

  // Allocate goal position value into byte array
  param_goal_position[0] = DXL_LOBYTE(index?DXL_MINIMUM_POSITION_VALUE:DXL_MAXIMUM_POSITION_VALUE);
  param_goal_position[1] = DXL_HIBYTE(index?DXL_MINIMUM_POSITION_VALUE:DXL_MAXIMUM_POSITION_VALUE);

  // Add Dynamixel#1 goal position value to the Syncwrite storage
  dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_goal_position);
  if (dxl_addparam_result != true)
  {
    fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL1_ID);
    return;
  }

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

bool Snake::configure(){
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

  return true;

}

bool start(){
	return true;
}
void stop(){

}
void cleanup(){

}