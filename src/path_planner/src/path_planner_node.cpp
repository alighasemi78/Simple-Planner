#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

void initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    geometry_msgs::Pose pose = msg->pose.pose;
    ROS_INFO("Received initial pose: x=%f, y=%f", pose.position.x, pose.position.y);
}

void goalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    geometry_msgs::Pose pose = msg->pose;
    ROS_INFO("Received goal pose: x=%f, y=%f", pose.position.x, pose.position.y);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planner_node");

    ros::NodeHandle init_pose_nh;
    ros::NodeHandle goal_pose_nh;

    ros::Subscriber init_pose_sub = init_pose_nh.subscribe("/initialpose", 1, initPoseCallback);
    ros::Subscriber goal_pose_sub = goal_pose_nh.subscribe("/move_base_simple/goal", 1, goalPoseCallback);

    ros::spin();
    return 0;
}
