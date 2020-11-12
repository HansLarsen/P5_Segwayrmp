#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <social_segway/Object.h>
#include <social_segway/ObjectList.h>
#include <social_segway/GetObjectsInRoom.h>
#include <social_segway/GetRooms.h>

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

int main(int argc, char **argv)
{

    ros::init(argc, argv, "semantic_node");
    ros::NodeHandle n;
    ros::ServiceClient getRoomsSrv = n.serviceClient<social_segway::GetRooms>("GetRooms");
    ros::ServiceClient getObjectsSrv = n.serviceClient<social_segway::GetObjectsInRoom>("GetObjectsInRoom");
    
    //Startup, get list of all rooms, get list of objects in each room
    std::vector<std::string> rooms;
    std::vector<std::vector<std::string>> objects;

    social_segway::GetRooms getRoomsMsg;
    


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
