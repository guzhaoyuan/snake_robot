# Snake Robot

a 7-DoF snake robot.

Robotis Dynamixel MX-28AR, Protocol1.0.

Ubuntu 16.04 LTS, ROS kinetic, ROS Control.

## Install Dynamixel

		# install ros kinetic

		# after insert usb2dynamixel
		sudo chmod 777 /dev/ttyUSB0
		
		# install dynamixel lib
		mkdir workspace && cd workspace
		git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
		cd DynamixelSDK/build/linux64
		make
		sudo make install
		
		# get code
		cd ~/workspace/src
		git clone https://github.com/guzhaoyuan/snake_robot.git
		cd ..
		catkin_make
	

Warning: before making any move, make sure the sorvo is not gonna interfere with anything to prevent damage or harm!!

## Design

![single DoF](meta/gif/single.gif)

![double DoF](meta/gif/double.gif)

![snake](meta/gif/snake.gif)

## Demo

- [x] ping servo.

		cd snake_robot/test/ping
		make
		./ping

- [x] Draw a circle using 2 joints.

		roslaunch snake_robot display_twoJointsSnake.launch 

![demo2](meta/pic/demo2.png)

- [ ] Draw a circle using 7 joint.
- [ ] Gravity Compensation, using known snake model and current pose state to apply force on each joint, thus to compensate the gravity efforts.

## TODO

- [ ] put the snake in gazebo to see how it does in real world
- [ ] IK for 2-DoF snake
- [ ] implement position and velocity interface
- [ ] drive one motor
- [ ] resource management

## Resources

- [eManual](http://support.robotis.com/en/product/actuator/dynamixel/mx_series/mx-28(2.0).htm)
- [Protocol 1.0](https://github.com/ROBOTIS-GIT/DynamixelSDK)
- [git manual](https://github.com/ROBOTIS-GIT/emanual/blob/master/docs/en/dxl/mx/mx-28-2.md)
- [drawing](https://github.com/ROBOTIS-GIT/emanual/blob/master/docs/en/dxl/mx/mx-28-2.md#drawings)
- [ros dynamixel tutorial](http://wiki.ros.org/dynamixel_controllers/Tutorials/Creating%20a%20joint%20torque%20controller)