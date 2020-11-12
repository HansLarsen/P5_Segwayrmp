#include <ros/ros.h>

/* NODE FLOW

    //PHASE 2 - search for changes
        Take input to which room there is a change in
        Search room for change, by taking a look at all known objects in room
        Compare found items with known items(comparator node)
        If an item has moved, add to list over changed items

        Always, if item is found, of same class as a changed item in the list
            grab info, and save the new position
        
        Loop until all rooms have been checked

        Goto phase 3

        //required functions
            
        
    // PHASE 3 - move items back
        Move to first item, ask user to place it on robot
        Move to default position, ask user to place it in correct position
        
*/

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "semantic_node");
    ros::NodeHandle n;






    return 0;
}
