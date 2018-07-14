#include <snakeHW.h>

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdio.h>

int getch()
{
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}

int kbhit(void)
{
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
#elif defined(_WIN32) || defined(_WIN64)
  return _kbhit();
#endif
}

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
	// Close port
  portHandler->closePort();
  std::cout<< "Leaving port closed\n";
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

void Snake::write(){
  
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
    printf("Press any key to terminate...\n");
    getch();
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
    printf("Press any key to terminate...\n");
    getch();
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

  return true;

}

bool start(){
	return true;
}
void stop(){

}
void cleanup(){

}