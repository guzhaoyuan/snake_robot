#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/JointState.h"


int main(int argc, char **argv){
	ros::init(argc, argv, "circleTwoJointsPublisher");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<sensor_msgs::JointState>("joint_states", 1000);

  ros::Rate loop_rate(20);

  int count = 0;
  double theta = 0;
  while(count<100 && ros::ok()){
  	count++;
  	theta += 3.1415926/100*2;

  	std_msgs::Header head;
  	sensor_msgs::JointState circlePoint;
  	head.seq = count;
  	head.stamp = ros::Time::now();
  	circlePoint.header = head;

  	circlePoint.name.push_back("joint1");
  	circlePoint.name.push_back("joint2");
  	circlePoint.position.push_back(sin(theta));
		circlePoint.position.push_back(cos(theta));
  	
  	pub.publish(circlePoint);
  	ros::spinOnce();//wait for callback

  	loop_rate.sleep();
  	if(count==100)
  		count=0;
  }
  return 0;
}