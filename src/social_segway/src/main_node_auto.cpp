#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <cameralauncher/Object.h>
#include <cameralauncher/ObjectList.h>
#include <social_segway/GetObjectsInRoom.h>
#include <social_segway/GetRooms.h>
#include <social_segway/CheckObject.h>
#include <social_segway/GetObjectPosById.h>
#include <iostream>
#include <string>
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <boost/algorithm/string.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include <vector>
#include <tf2_ros/transform_listener.h>
#include <actionlib/client/simple_action_client.h>

#define NODE_NAME "[Main_node]: "
#define MAX_OBJECT_DISTANCE 2.0
#define DEGREES_TO_RADIANS 0.01745329
#define RADIANS_TO_DEGREES 57.2957878

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

ros::Publisher * marker_pub; 

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

void convert(const geometry_msgs::Transform &trans, geometry_msgs::Pose &pose)
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

class ObjectData : public cameralauncher::Object
{
public:
    bool object_at_default = false;
    ObjectData(cameralauncher::Object oldObject) {
        this->id = oldObject.id;
        this->objectClass = oldObject.objectClass;
        this->transform = oldObject.transform;
        this->type = oldObject.type;
        this->object_at_default = false;
    }

    ObjectData(int32_t id, geometry_msgs::Transform transform) {
        this->id = id;
        this->transform = transform;
        this->objectClass = "";
        this->type = "";
        this->object_at_default = false;
    }
};

class RoomData
{
public:
    std::vector<ObjectData> objects;
    std::string name;
    bool room_visited;
};

void pub_marker_single(ObjectData object, ros::Publisher targetObjectPub) {
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
}

bool check_angle_dist_to_target(tf2_ros::Buffer * tfBuffer, ObjectData object) {
    geometry_msgs::TransformStamped robotTransform;
    try
    {
        robotTransform = tfBuffer->lookupTransform("map", "base_link",
                                                    ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        return false;
    }
    //ROS_INFO_STREAM(robotTransform);
    //ROS_INFO_STREAM(object.transform);

    float distance = distanceBetweenTransforms(robotTransform.transform, object.transform);
    if (distance > MAX_OBJECT_DISTANCE)
    {
        ROS_INFO_STREAM("Please drive closer to object, distance: " << distance);
        return false;
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
    convert(tempTransform, pose);

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
        ROS_INFO_STREAM(NODE_NAME << object.objectClass << " in range (" << distance << "), but not facing it yet (angle=" << angle * RADIANS_TO_DEGREES << " degrees)");
        ros::Duration(1.0).sleep();
        return false;
    }

    ROS_INFO_STREAM(NODE_NAME << "Got" << object.objectClass << " in sight, distance: " << distance << ", angle: " << angle * RADIANS_TO_DEGREES << "\n");

    visualization_msgs::Marker marker = visualization_msgs::Marker();
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "main_node";
    marker.id = 2;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Point pointZero;
    pointZero.x = object.transform.translation.x;
    pointZero.y = object.transform.translation.y;
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
    marker.lifetime = ros::Duration(10);
    
    marker_pub->publish(marker);

    //object in range, and we can see it
    
    return true;
}

bool check_dist_to_target(tf2_ros::Buffer * tfBuffer, ObjectData object) {
    geometry_msgs::TransformStamped robotTransform;
    try
    {
        robotTransform = tfBuffer->lookupTransform("map", "base_link", ros::Time(0), ros::Duration(0.5));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        return false;
    }
    //ROS_INFO_STREAM(robotTransform);
    //ROS_INFO_STREAM(object.transform);

    float distance = distanceBetweenTransforms(robotTransform.transform, object.transform);
    if (distance < MAX_OBJECT_DISTANCE)
    {
        //ROS_INFO_STREAM("Please drive closer to object, distance: " << distance);
        return true;
    }

    visualization_msgs::Marker marker = visualization_msgs::Marker();
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "main_node";
    marker.id = 2;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Point pointZero;
    pointZero.x = object.transform.translation.x;
    pointZero.y = object.transform.translation.y;
    pointZero.z = 0;
    marker.points.push_back(pointZero);

    geometry_msgs::Point pointEnd;
    pointEnd.x = robotTransform.transform.translation.x;
    pointEnd.y = robotTransform.transform.translation.y;
    pointEnd.z = 0;
    marker.points.push_back(pointEnd);
    
    marker.scale.x = 0.05;
    marker.scale.y = 0.07;
    marker.color.a = 0.0;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 1.0;
    marker.lifetime = ros::Duration(10);
    
    marker_pub->publish(marker);

    //object in range, and we can see it
    
    return false;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "Main_node");
    ros::NodeHandle n;
    ros::Publisher targetObjectPub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
    ros::ServiceClient getRoomsSrv = n.serviceClient<social_segway::GetRooms>("/get_rooms");
    ros::ServiceClient getObjectsSrv = n.serviceClient<social_segway::GetObjectsInRoom>("/get_objects_in_room");
    ros::ServiceClient getCheckObjectSrv = n.serviceClient<social_segway::CheckObject>("/check_object");
    ros::ServiceClient getObjectPosById = n.serviceClient<social_segway::GetObjectPosById>("/get_location_by_id");
    ros::Publisher marker_pub_holder = n.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
    marker_pub = &marker_pub_holder;


    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base_navi", true);
    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::TransformStamped transformRobotGoal;

    RoomData roomData[25];
    int roomDataSize = 0;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    bool creatingMap = false;

    //wait for the action server to come up
    while (!ac.waitForServer(ros::Duration(5.0)) && ros::ok())
    {
        ROS_INFO("Waiting for the move_base action server to come up");
        ros::spinOnce();
    }

    //Startup, get list of all rooms, get list of objects in each room
    social_segway::GetRooms getRoomsMsg;
    do {
        getRoomsSrv.call(getRoomsMsg);
        if (getRoomsMsg.response.rooms.size() == 0)
        {
            ROS_WARN_STREAM(NODE_NAME << "No rooms gotten from semantic node");
            ROS_WARN_STREAM(NODE_NAME << "Create a map before continuing");
            ros::Duration(5).sleep(); 
            creatingMap = true;
        }
        else {
            ROS_INFO_STREAM(NODE_NAME << "Map found continuing!");
            creatingMap = false;
        }
    }
    while(creatingMap);

    int numObjects = 0;
    for (auto roomName : getRoomsMsg.response.rooms)
    {
        social_segway::GetObjectsInRoom getObjSrv;
        getObjSrv.request.room = roomName;
        getObjectsSrv.call(getObjSrv);

        RoomData data;
        data.name = roomName;
        for (cameralauncher::Object workingOjbect : getObjSrv.response.objects)
        {
            data.objects.push_back(workingOjbect);
        }
        data.room_visited = false;

        roomData[roomDataSize] = data;
        roomDataSize++;
        numObjects += getObjSrv.response.objects.size();
    }

    bool rooms_done = false;
    while (!rooms_done)
    {
        ROS_INFO_STREAM(NODE_NAME << "got " << roomDataSize << " rooms, with a total of " << numObjects << " objects. \n Type a number out of range to end it.\n");
        for (RoomData workingRoom : roomData) {
            ROS_INFO_STREAM(workingRoom.name << " " << workingRoom.room_visited);
        }
        ROS_INFO_STREAM(NODE_NAME << "Ready to search rooms. In which should I search?");

        for (int i = 0; i < roomDataSize; i++)
        {
            std::cout << "(" << i << ") " << roomData[i].name << ", ";
        }
        std::cout << "\n";
        char answer;
        std::cin >> answer;
        int curRoomNum = (int)answer - 48;

        if (curRoomNum > roomDataSize){
            break;
        }

        auto curRoom = roomData[curRoomNum];

        for (auto object : curRoom.objects)
        {
            pub_marker_single(object, targetObjectPub);

            bool reached = false;
            while (!reached)
            {
                geometry_msgs::TransformStamped transform;
                
                transform.header.stamp = ros::Time::now();
                transform.header.frame_id = "map";
                transform.child_frame_id = "goal";
                transform.transform.translation = object.transform.translation;
                transform.transform.rotation.w = 1;

                br.sendTransform(transform);

                try{
                    transformStamped = tfBuffer.lookupTransform("odom", "goal", transform.header.stamp, ros::Duration(1.0));
                    transformRobotGoal = tfBuffer.lookupTransform("base_link", "goal", transform.header.stamp, ros::Duration(1.0));
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
                goal.target_pose.pose.orientation = transformRobotGoal.transform.rotation;

                ROS_INFO("Sending goal");
                ac.sendGoal(goal);

                while (!check_dist_to_target(&tfBuffer, object)) {

                    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                        ROS_INFO("Hooray, got to the object.");
                        reached = true;
                        break;
                    }
                    ros::spinOnce();
                    //ROS_INFO_STREAM(NODE_NAME << ac.getState());
                    ros::Duration(1).sleep();
                }

                if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                    ROS_INFO("Hooray, got to the object.");
                    reached = true;
                }
                else {
                    ROS_INFO("The movement failed.");
                    ac.cancelAllGoals();
                }

                reached = true;

                social_segway::CheckObject checkingObjectMSG;
                checkingObjectMSG.request.id = object.id;

                getCheckObjectSrv.call(checkingObjectMSG);

                if (checkingObjectMSG.response.success) {
                    ROS_INFO_STREAM(NODE_NAME << object.objectClass <<" detect at id:" << object.id << " in room: " << curRoomNum << std::endl);
                    object.object_at_default = true;
                }
                else
                {
                    ROS_WARN_STREAM(NODE_NAME << object.objectClass << " not detect found at id:" << object.id << " in room: " << curRoomNum << std::endl);
                    object.object_at_default = false;
                }
                ros::Duration(1.0).sleep();
            }
        } // All objects in the room has been looked at, list the changed objects.

        curRoom.room_visited = true;
        if (curRoomNum >= roomDataSize){
            rooms_done = true;
        }
    }

    ROS_INFO_STREAM("-------------------HEJ-------------");

    for (int i = 0; i > roomDataSize; i++) {
        auto workingRoom = roomData[i];
        ROS_INFO_STREAM(workingRoom.name);
        if (workingRoom.room_visited) {
            ROS_INFO_STREAM("---------------- Current Room : " << workingRoom.name << " -------------------------");
            for (ObjectData workingObject : workingRoom.objects) {
                ROS_INFO_STREAM(NODE_NAME << "Object at default: " << workingObject.object_at_default << " Objecttype: " << workingObject.type);
            }
        }
    }

    ROS_INFO_STREAM("HEJ 2");

    ObjectData * object_move_to_default;
    for (RoomData workingRoom : roomData) {
        for (ObjectData workingObject : workingRoom.objects) {
            boost::to_lower(workingObject.type);
            if (workingObject.type  == "furniture") {
                continue;
            }

            if (workingObject.object_at_default == false) {
                object_move_to_default = &workingObject;
                break;
            }
        }
    }


    social_segway::GetObjectPosById checkingObjectMSG;
    checkingObjectMSG.request.id = object_move_to_default->id;

    if (getObjectPosById.call(checkingObjectMSG) == true) {
        if (checkingObjectMSG.response.success) {
            pub_marker_single(*object_move_to_default, targetObjectPub);
            ROS_INFO_STREAM(NODE_NAME << "Goto first " << object_move_to_default->objectClass);
        }
        else
        {
            ROS_ERROR_STREAM(NODE_NAME << "EPIC FAIL");
            return 0;
        }

    }
    else
    {
        ROS_ERROR_STREAM(NODE_NAME << "EPIC FAIL");
        return 0;
    }

    ObjectData newTmp = *object_move_to_default;
    newTmp.transform.translation = checkingObjectMSG.response.translation;

    while(!check_angle_dist_to_target(&tfBuffer, newTmp)) {
        ROS_INFO_STREAM(NODE_NAME << "Move me closer so that i may hit them with my sword!");
        ros::Duration(1).sleep();
    }

    ROS_INFO_STREAM(NODE_NAME << "Put the object infront of me ontop of me!");

    while(!check_angle_dist_to_target(&tfBuffer, *object_move_to_default)) {
        ROS_INFO_STREAM(NODE_NAME << "Move me closer so that i may hit them with my sword!");
        ros::Duration(1).sleep();
    }

    /*
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> client("do_dishes", true); // true -> don't need ros::spin()
    client.waitForServer();
    move_base_msgs::MoveBaseGoal goal;
    
    client.sendGoal(goal);
    //client.waitForResult(ros::Duration(5.0));
    //if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    */

    ROS_INFO_STREAM(NODE_NAME << "Done ski!");
    return 0;
}
