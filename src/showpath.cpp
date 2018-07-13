// created by zion on 2018/7/12
// this is a separate function to showpath, it will listen to tf for a while and start pub to rviz to show path.
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>


main (int argc, char **argv)
{

    if(argc < 2){//meaning that show path should use param to show link
        return -1;
    }
    std::string endLink = argv[1];

    ros::init (argc, argv, "showpath");

    ros::NodeHandle ph;
    ros::Publisher path_pub = ph.advertise<nav_msgs::Path>("trajectory", 1, true);

    // listener store for 3s of tf data
    tf::TransformListener listener(ros::Duration(3));
    // listen for 1s
    ros::Duration(1).sleep();

    nav_msgs::Path path;

    while (ros::ok())
    {
        ros::Time current_time = ros::Time(0);

        std_msgs::Header head2;
        head2.frame_id = "base_link";
        head2.stamp = current_time;
        path.header = head2;

        geometry_msgs::PoseStamped relative_pose;
        relative_pose.header.frame_id = endLink;
        relative_pose.header.stamp = current_time;
        relative_pose.pose.position.z = 0.072;// distance to link3 coordinate
        relative_pose.pose.orientation.w = 1;

        geometry_msgs::PoseStamped base_pose;
        // transform relative pose to global pose
        listener.transformPose("base_link", relative_pose, base_pose);

        path.poses.push_back(base_pose);

        // clear path tail
        if(path.poses.size()>3000)
            path.poses.erase(path.poses.begin(), path.poses.begin()+500);
        path_pub.publish(path);
    }

    return 0;
}