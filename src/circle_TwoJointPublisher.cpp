#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/JointState.h"

#define updateRate 20
#define pi 3.1415926
#define precision 100

int main(int argc, char **argv){
	ros::init(argc, argv, "circleTwoJointsPublisher");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<sensor_msgs::JointState>("joint_states", 1000);

  ros::Rate loop_rate(updateRate);

  int count = 0;
  double theta = 0;
  while(count < precision && ros::ok()){
  	count++;
  	theta += pi*2/precision;

  	std_msgs::Header head;
  	sensor_msgs::JointState circlePoint;
  	head.stamp = ros::Time::now();
  	circlePoint.header = head;

  	circlePoint.name.push_back("joint1");
  	circlePoint.name.push_back("joint2");
  	circlePoint.position.push_back(sin(theta));
		circlePoint.position.push_back(cos(theta));
  	
  	pub.publish(circlePoint);
  	ros::spinOnce();//wait for callback

  	loop_rate.sleep();

  	//to loop over and over again
  	if(count == precision)
  		count = 0;
  }
  return 0;
}