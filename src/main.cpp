#include <snakeHW.h>
#include <iostream>

int main()
{
	Snake snake;
	// controller_manager::ControllerManager cm(&snake);
	int i = 0;
	while (true)
	{
		snake.read();
		//cm.update(snake.get_time(), snake.get_period());
		snake.write();
		std::cout << "round " << ++i << std::endl;
		sleep(1);
	}
	return 0;
}
