// created by zion on 2018/7/31
// a test to use service from moveit's move_group to compute FK and IK

#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// Robot state publishing
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>

// Kinematics
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/GetPositionFK.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ik_client");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle node_handle;
  ros::ServiceClient FK_client = node_handle.serviceClient<moveit_msgs::GetPositionFK>("compute_fk");
  ros::ServiceClient IK_client = node_handle.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");

  while (!IK_client.exists())
  {
    ROS_INFO("Waiting for service");
    sleep(1.0);
  }

// init robot model loader
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
// get robotmodel from loader
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
// get robotstate from robotmodel
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
// get jointgroup from robotmodel, group is defined by moveit setup assistant
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("three_joint");

// FK request -------------------------------------------
  moveit_msgs::GetPositionFK::Request FK_request;
  moveit_msgs::GetPositionFK::Response FK_response;

  FK_request.header.stamp = ros::Time(0);
  FK_request.header.frame_id = "base_link";

  FK_request.fk_link_names.push_back("base_link");
  FK_request.fk_link_names.push_back("link2");
  FK_request.fk_link_names.push_back("link3");
  FK_request.fk_link_names.push_back("link4");

  FK_request.robot_state.joint_state.name.push_back("joint1");
  FK_request.robot_state.joint_state.name.push_back("joint2");
  FK_request.robot_state.joint_state.name.push_back("joint3");
  FK_request.robot_state.joint_state.position.push_back(0.0);
  FK_request.robot_state.joint_state.position.push_back(0.0);
  FK_request.robot_state.joint_state.position.push_back(0.0);

  FK_client.call(FK_request, FK_response);
  ROS_INFO_STREAM(
      "Result: " << ((FK_response.error_code.val == FK_response.error_code.SUCCESS) ? "True " : "False ")
                 << FK_response.error_code.val);

// IK request -------------------------------------------
  moveit_msgs::GetPositionIK IK_srv;

  IK_srv.request.ik_request.group_name = "three_joint";

  // joint state
  IK_srv.request.ik_request.robot_state.joint_state.header.stamp = ros::Time(0);
  IK_srv.request.ik_request.robot_state.joint_state.header.frame_id = "base_link";
  IK_srv.request.ik_request.robot_state.joint_state.name.push_back("joint1");
  IK_srv.request.ik_request.robot_state.joint_state.name.push_back("joint2");
  IK_srv.request.ik_request.robot_state.joint_state.name.push_back("joint3");
  IK_srv.request.ik_request.robot_state.joint_state.position.push_back(0.0);
  IK_srv.request.ik_request.robot_state.joint_state.position.push_back(0.0);
  IK_srv.request.ik_request.robot_state.joint_state.position.push_back(0.0);
  // pose stamp
  IK_srv.request.ik_request.pose_stamped = FK_response.pose_stamped[3];
  ROS_DEBUG_STREAM(FK_response.pose_stamped[3].pose.position.x<<FK_response.pose_stamped[3].pose.position.y<<FK_response.pose_stamped[3].pose.position.z);
  ROS_DEBUG_STREAM(FK_response.pose_stamped[3].pose.orientation.x<<FK_response.pose_stamped[3].pose.orientation.y<<FK_response.pose_stamped[3].pose.orientation.z<<FK_response.pose_stamped[3].pose.orientation.w);
  // IK_srv.request.ik_request.pose_stamped.header.stamp = ros::Time(0);
  // IK_srv.request.ik_request.pose_stamped.header.frame_id = "base_link";

  // IK_srv.request.ik_request.pose_stamped.pose.position.x = FK_response.pose_stamped[0].pose.position.x;
  // IK_srv.request.ik_request.pose_stamped.pose.position.y = FK_response.pose_stamped[0].pose.position.y;
  // IK_srv.request.ik_request.pose_stamped.pose.position.z = FK_response.pose_stamped[0].pose.position.z;

  // IK_srv.request.ik_request.pose_stamped.pose.orientation.x = FK_response.pose_stamped[0].pose.orientation.x;
  // IK_srv.request.ik_request.pose_stamped.pose.orientation.y = FK_response.pose_stamped[0].pose.orientation.y;
  // IK_srv.request.ik_request.pose_stamped.pose.orientation.z = FK_response.pose_stamped[0].pose.orientation.z;
  // IK_srv.request.ik_request.pose_stamped.pose.orientation.w = FK_response.pose_stamped[0].pose.orientation.w;

	IK_client.call(IK_srv);
  if(IK_srv.response.error_code.val == IK_srv.response.error_code.SUCCESS){
    ROS_INFO_STREAM("Joints:"<< IK_srv.response.solution.joint_state.position[0]
                            << IK_srv.response.solution.joint_state.position[1]
                            << IK_srv.response.solution.joint_state.position[2]);
  }else{
    ROS_INFO_STREAM(
      "Result: " << ((IK_srv.response.error_code.val == IK_srv.response.error_code.SUCCESS) ? "True " : "False ")
                 << IK_srv.response.error_code.val);
  }

}