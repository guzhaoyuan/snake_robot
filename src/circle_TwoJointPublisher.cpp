// created by zion on 2018/7/12
// it write to 2 joint snake and make snake end goes a ellipse
#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"

#include <nav_msgs/Path.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#define updateRate 200
#define pi 3.1415926
#define precision 1000


int main(int argc, char **argv){
	ros::init(argc, argv, "circleTwoJointsPublisher");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<sensor_msgs::JointState>("joint_states", 1000);

  ros::Publisher joint1_pub = n.advertise<std_msgs::Float64>("/snake/joint1_position_controller/command", 1000);
  ros::Publisher joint2_pub = n.advertise<std_msgs::Float64>("/snake/joint2_position_controller/command", 1000);

  ros::Rate loop_rate(updateRate);

  int count = 0;
  double theta = 0;

  while(ros::ok()){
  	count++;
  	theta += pi*2/precision;

    // pack position
  	std_msgs::Header head;
  	sensor_msgs::JointState circlePoint;
  	head.stamp = ros::Time::now();
  	circlePoint.header = head;
    //calc point
  	circlePoint.name.push_back("joint1");
  	circlePoint.name.push_back("joint2");
  	circlePoint.position.push_back(0.5*sin(theta));
		circlePoint.position.push_back(0.5*cos(theta));
  	
  	pub.publish(circlePoint);

    std_msgs::Float64 msg;
    msg.data = 0.5*sin(theta);
    joint1_pub.publish(msg);

    msg.data = 0.5*cos(theta);
    joint2_pub.publish(msg);

  	ros::spinOnce();//check for incoming messages, wait for callback

  	loop_rate.sleep();

  	//to loop over and over again
  	if(!count%precision){
      theta = 0;
    }
  }
  return 0;
}
