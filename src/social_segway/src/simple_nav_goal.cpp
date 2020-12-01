#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "geometry_msgs/PointStamped.h"
#include <vector>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

std::vector<geometry_msgs::PointStamped> my_points;

void callback(const geometry_msgs::PointStamped::ConstPtr &point){
    my_points.push_back(*point);
    ROS_INFO("GOT POINT!");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_navigation_goals");
    ros::NodeHandle node;

    ros::Subscriber sub = node.subscribe("/clicked_point", 1, callback);
    int working_index = 0;

    while (ros::ok()) {
        if (my_points.size() < 4) {
            ros::Duration(1.0).sleep();
            ros::spinOnce();
            ROS_INFO("Waiting cakes");
            continue;
        }

        //tell the action client that we want to spin a thread by default
        MoveBaseClient ac("move_base", true);

        //wait for the action server to come up
        while (!ac.waitForServer(ros::Duration(5.0)) && ros::ok())
        {
            ros::spinOnce();
            ROS_INFO("Waiting for the move_base action server to come up");
        }

        move_base_msgs::MoveBaseGoal goal;

        //we'll send a goal to the robot to move 1 meter forward
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = my_points[working_index].point.x;
        goal.target_pose.pose.position.y = my_points[working_index].point.y;
        goal.target_pose.pose.position.z = 0;
        goal.target_pose.pose.orientation.w = 1.0;

        ROS_INFO("Sending goal");
        ac.sendGoal(goal);

        ac.waitForResult();

        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Hooray, the base moved 1 meter forward");
        else
            ROS_INFO("The base failed to move forward 1 meter for some reason");

        working_index++;
        if (working_index > 3) {
            working_index = 0;
        }
    }

    return 0;
}