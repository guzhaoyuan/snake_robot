#include <snakeHW.h>
#include "ros/ros.h"
#include <iostream>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "snakeHW");
	ros::NodeHandle n;

	Snake snake;
	// controller_manager::ControllerManager cm(&snake);

	int i = 0;
	while (ros::ok())
	{
		snake.read();
		//cm.update(snake.get_time(), snake.get_period());
		snake.write();
		std::cout << "round " << ++i << std::endl;
		ros::Duration(1).sleep();
	}
	return 0;
}
