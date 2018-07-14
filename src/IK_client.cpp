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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ik_client");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle node_handle;
  ros::ServiceClient service_client = node_handle.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");

  while (!service_client.exists())
  {
    ROS_INFO("Waiting for service");
    sleep(1.0);
  }

  moveit_msgs::GetPositionIK::Request service_request;
  moveit_msgs::GetPositionIK::Response service_response;

  service_request.ik_request.group_name = "three_joint";
  service_request.ik_request.pose_stamped.header.frame_id = "base_link";
  service_request.ik_request.pose_stamped.pose.position.x = 0.;
  service_request.ik_request.pose_stamped.pose.position.y = 0.;
  service_request.ik_request.pose_stamped.pose.position.z = 0.144;

  service_request.ik_request.pose_stamped.pose.orientation.x = 0.0;
  service_request.ik_request.pose_stamped.pose.orientation.y = 0.0;
  service_request.ik_request.pose_stamped.pose.orientation.z = 0.0;
  service_request.ik_request.pose_stamped.pose.orientation.w = 1.0;

  service_client.call(service_request, service_response);

  ROS_INFO_STREAM(
      "Result: " << ((service_response.error_code.val == service_response.error_code.SUCCESS) ? "True " : "False ")
                 << service_response.error_code.val);

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("three_joint");

  service_request.ik_request.robot_state.joint_state.name = joint_model_group->getJointModelNames();
  kinematic_state->setToRandomPositions(joint_model_group);
  kinematic_state->copyJointGroupPositions(joint_model_group,
                                           service_request.ik_request.robot_state.joint_state.position);

	service_client.call(service_request, service_response);
  ROS_INFO_STREAM(
      "Result: " << ((service_response.error_code.val == service_response.error_code.SUCCESS) ? "True " : "False ")
                 << service_response.error_code.val);

}