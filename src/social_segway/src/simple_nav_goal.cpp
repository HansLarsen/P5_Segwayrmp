#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <actionlib/client/simple_action_client.h>
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include <vector>
#include <tf2_ros/transform_listener.h>

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


    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base_navi", true);
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;


    //wait for the action server to come up
    while (!ac.waitForServer(ros::Duration(5.0)) && ros::ok())
    {
        ROS_INFO("Waiting for the move_base action server to come up");
        ros::spinOnce();
    }

    while(ros::ok()) {
        if (my_points.size() < 4) {
            ros::Duration(1.0).sleep();
            ros::spinOnce();
            ROS_INFO("Waiting for goals");
            continue;
        }
        break;
    }

    ROS_INFO("Got action server");
    ros::Rate rate(10.0);
    while (ros::ok()) {
        geometry_msgs::TransformStamped transform;

        transform.header.stamp = ros::Time::now();
        transform.header.frame_id = "map";
        transform.child_frame_id = "goal";
        transform.transform.translation.x = my_points[working_index].point.x;
        transform.transform.translation.y = my_points[working_index].point.y;
        transform.transform.translation.z = 0;
        transform.transform.rotation.w = 1;

        br.sendTransform(transform);

        try{
            transformStamped = tfBuffer.lookupTransform("odom", "goal", transform.header.stamp, ros::Duration(2.0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        move_base_msgs::MoveBaseGoal goal;

        //we'll send a goal to the robot to move 1 meter forward
        goal.target_pose.header.frame_id = transformStamped.header.frame_id;
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = transformStamped.transform.translation.x;
        goal.target_pose.pose.position.y = transformStamped.transform.translation.y;
        goal.target_pose.pose.position.z = transformStamped.transform.translation.z;
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
        rate.sleep();
    }

    return 0;
}