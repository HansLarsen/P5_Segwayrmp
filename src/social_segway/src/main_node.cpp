#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <social_segway/Object.h>
#include <social_segway/ObjectList.h>
#include <social_segway/GetObjectsInRoom.h>
#include <social_segway/GetRooms.h>
#include <iostream>
#include <string>
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>

#define NODE_NAME "[Main_node]: "
#define MAX_OBJECT_DISTANCE 2.0
#define DEGREES_TO_RADIANS 0.01745329
#define RADIANS_TO_DEGREES 57.2957878

/* NODE FLOW
    //PHASE 2 - search for changes
        Take input to which room there is a change in
        Search room for change, by taking a look at all known objects in room
            By publishing markers the user will drive the robot to
        Compare found items with known items(comparator node)
        If an item has moved, add to list over changed items

        Always, if item is found, of same class as a changed item in the list
            grab info, and save the new position
        
        Loop until all rooms have been checked

        Goto phase 3

        //required functions
        - take input from user
        - move to a position, to look at object
            - find viable positions
            - choose a position
            - use move_base to go there
            - rotate segway to face object (or at an offset, if so wanted by cameras)
        - check if semantic node has found the changed object
        - grab image of changed item
        
        
    // PHASE 3 - move items back
        Move to first item, ask user to place it on robot
        Move to default position, ask user to place it in correct position

        //required functions
        - move to a position close to object
        - ask user to do stuff
*/

/* TEST DESIGN
    First phase, manually drive robot around with semantic_node running, in mapper mode.

    PHASE 2:
    get default item map from semantic_node
    ask for room with change

    for each room with change:
      point: a
        find closest object
        publish marker at object position
        request user to move robot such that it is within 1.5m of marker, looking at the object(within front cameraes' FOV)
        wait a small amount of time
        ask semantic_node for map of changes
        if object is gone, save that info
        if other object is present, compare with previously known moved items => have we found the item again? => then match up the id's

        while room has more objects:
            goto point: a

    
*/

void convertTransToPose(const geometry_msgs::Transform &trans, geometry_msgs::Pose &pose)
{
    pose.orientation = trans.rotation;
    pose.position.x = trans.translation.x;
    pose.position.y = trans.translation.y;
    pose.position.z = trans.translation.z;
}

float distanceBetweenTransforms(geometry_msgs::Transform &a, geometry_msgs::Transform &b)
{
    tf2::Vector3 distVector;
    distVector.setX(a.translation.x - b.translation.x);
    distVector.setY(a.translation.y - b.translation.y);
    distVector.setZ(0); //distance in x-y plane only
    return distVector.length();
}

float angleBetweenVectors(tf2::Vector3 &a, tf2::Vector3 &b)
{
    auto dot = a.dot(b);
    return acos(dot / (a.length() * b.length()));
}

struct RoomData
{
    std::vector<social_segway::Object> objects;
    std::string name;
    bool visited;
};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "Main_node");
    ros::NodeHandle n;
    ros::Publisher targetObjectPub = n.advertise<visualization_msgs::MarkerArray>("target_object", 10);
    ros::ServiceClient getRoomsSrv = n.serviceClient<social_segway::GetRooms>("/get_rooms");
    ros::ServiceClient getObjectsSrv = n.serviceClient<social_segway::GetObjectsInRoom>("/get_objects_in_room");
    std::vector<RoomData> roomData;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    //Startup, get list of all rooms, get list of objects in each room
    social_segway::GetRooms getRoomsMsg;
    getRoomsSrv.call(getRoomsMsg);
    if (getRoomsMsg.response.rooms.size() == 0)
    {
        ROS_ERROR_STREAM(NODE_NAME << "No rooms gotten from semantic node, exiting");
        exit(0);
    }

    int numObjects = 0;
    for (auto roomName : getRoomsMsg.response.rooms)
    {
        social_segway::GetObjectsInRoom getObjSrv;
        getObjSrv.request.room = roomName;
        getObjectsSrv.call(getObjSrv);

        RoomData data;
        data.name = roomName;
        data.objects = getObjSrv.response.objects;
        data.visited = false;

        roomData.push_back(data);
        numObjects += getObjSrv.response.objects.size();
    }

    ROS_INFO_STREAM(NODE_NAME << "got " << roomData.size() << " rooms, with a total of " << numObjects << " objects");
    ROS_INFO_STREAM(NODE_NAME << "Ready to search rooms. I know the following rooms, in which should I search first?");

    for (int i = 0; i < roomData.size(); i++)
    {
        std::cout << "(" << i << ") " << roomData[i].name << ", ";
    }
    std::cout << "\n";
    char answer;
    std::cin >> answer;
    int curRoomNum = (int)answer - 48;

    bool done = false;
    while (!done)
    {
        auto curRoom = roomData[curRoomNum];

        for (auto object : curRoom.objects)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time();
            marker.ns = "main_node";
            marker.id = 1;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = object.transform.translation.x;
            marker.pose.position.y = object.transform.translation.y;
            marker.pose.position.z = object.transform.translation.z;
            marker.pose.orientation.x = object.transform.rotation.x;
            marker.pose.orientation.y = object.transform.rotation.y;
            marker.pose.orientation.z = object.transform.rotation.z;
            marker.pose.orientation.w = object.transform.rotation.w;
            marker.scale.x = 0.5;
            marker.scale.y = 0.5;
            marker.scale.z = 0.5;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            marker.lifetime = ros::Duration(0);
            visualization_msgs::MarkerArray msg;
            msg.markers.push_back(marker);
            targetObjectPub.publish(msg);

            bool reached = false;
            while (!reached)
            {
                geometry_msgs::TransformStamped robotTransform;
                try
                {
                    robotTransform = tfBuffer.lookupTransform("map", "base_link",
                                                              ros::Time(0));
                }
                catch (tf2::TransformException &ex)
                {
                    ROS_WARN("%s", ex.what());
                    ros::Duration(1.0).sleep();
                    continue;
                }
                ROS_INFO_STREAM(robotTransform);
                ROS_INFO_STREAM(object.transform);

                float distance = distanceBetweenTransforms(robotTransform.transform, object.transform);
                if (distance > MAX_OBJECT_DISTANCE)
                {
                    ROS_INFO_STREAM("Please drive closer to object, distance: " << distance);
                    ros::Duration(1.0).sleep();
                    continue;
                }

                /*
                tf2::Quaternion q1;
                q1.setX(robotTransform.transform.rotation.x);
                q1.setY(robotTransform.transform.rotation.y);
                q1.setZ(robotTransform.transform.rotation.z);
                q1.setW(robotTransform.transform.rotation.w);
                */
                auto tempTransform = robotTransform.transform;
                tempTransform.translation.z = 0;
                tf2::Transform pose; // = robotTransform.transform.rotation; // make this work
                convertTransToPose(tempTransform, pose);

                //ROS_INFO_STREAM(pose.getOrigin().x() << ", " << pose.getOrigin().y() << ", " << pose.getOrigin().z());
                //ROS_INFO_STREAM(pose.getRotation().x() << ", " << pose.getRotation().y() << ", " << pose.getRotation().z() << ", " << pose.getRotation().w());
                tf2::Vector3 x_axis(1, 0, 0);
                tf2::Vector3 robot_x_axis = pose.getBasis() * x_axis;
                //ROS_INFO_STREAM(robot_x_axis.x() << ", " << robot_x_axis.y() << ", " << robot_x_axis.z());

                tf2::Vector3 object_pos;

                object_pos.setX(object.transform.translation.x);
                object_pos.setY(object.transform.translation.y);
                object_pos.setZ(0);

                float angle = angleBetweenVectors(robot_x_axis, object_pos);
                if (abs(angle) > 45.0 * DEGREES_TO_RADIANS)
                {
                    ROS_INFO_STREAM("Object in range (" << distance << "), but not facing it yet (angle=" << angle * RADIANS_TO_DEGREES << " degrees)");
                    ros::Duration(1.0).sleep();
                    continue;
                }
                /*
                marker = visualization_msgs::Marker();
                marker.header.frame_id = "map";
                marker.header.stamp = ros::Time();
                marker.ns = "main_node";
                marker.id = 2;
                marker.type = visualization_msgs::Marker::ARROW;
                marker.action = visualization_msgs::Marker::ADD;
                geometry_msgs::Point pointZero;
                pointZero.x = 0;
                pointZero.y = 0;
                pointZero.z = 0;
                marker.points.push_back(pointZero);
                geometry_msgs::Point pointEnd;
                pointEnd.x = robot_x_axis.x();
                pointEnd.y = robot_x_axis.y();
                pointEnd.z = 0;
                marker.points.push_back(pointEnd);
                
                marker.scale.x = 0.05;
                marker.scale.y = 0.07;
                marker.color.a = 1.0;
                marker.color.r = 0.0;
                marker.color.g = 0.0;
                marker.color.b = 1.0;
                marker.lifetime = ros::Duration(0);
                msg.markers.push_back(marker);
                marker.id = 3;
                marker.points[1].x = object.transform.translation.x;
                marker.points[1].y = object.transform.translation.y;
                marker.points[1].z = 0;
                marker.color.b = 0.0;
                marker.color.g = 1.0;
                msg.markers.push_back(marker);
                targetObjectPub.publish(msg);
                */
                //object in range, and we can see it

                ROS_INFO_STREAM("Got object in sight, distance: " << distance << ", angle: " << angle * RADIANS_TO_DEGREES);
                ROS_INFO_STREAM("\n\n\n");
                reached = true;
                ros::Duration(1.0).sleep();
            }
        }
        done = true;
    }

    /*
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> client("do_dishes", true); // true -> don't need ros::spin()
    client.waitForServer();
    move_base_msgs::MoveBaseGoal goal;
    
    client.sendGoal(goal);
    //client.waitForResult(ros::Duration(5.0));
    //if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    */

    return 0;
}
