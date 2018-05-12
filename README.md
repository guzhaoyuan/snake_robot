# Snake Robot

a 7-DoF snake robot, using dynamixel MX-28AR, ROS Control.

## Design

![single DoF](meta/gif/single.gif)

![double DoF](meta/gif/double.gif)

![snake](meta/gif/snake.gif)

## Demo

Gravity Compensation, using known snake model and current pose state to apply force on each joint, thus to compensate the gravity efforts.

## TODO

- [ ] implement position and velocity interface
- [ ] drive one motor
- [ ] resource management

## Resources

- [eManual](http://support.robotis.com/en/product/actuator/dynamixel/mx_series/mx-28(2.0).htm)
- [Protocol 1.0](https://github.com/ROBOTIS-GIT/DynamixelSDK)
- [git manual](https://github.com/ROBOTIS-GIT/emanual/blob/master/docs/en/dxl/mx/mx-28-2.md)
- [drawing](https://github.com/ROBOTIS-GIT/emanual/blob/master/docs/en/dxl/mx/mx-28-2.md#drawings)
