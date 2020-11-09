#include "ros/ros.h"
#include "semantic_map/Rvizcommand.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rvizcommand");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<semantic_map::Rvizcommand>("rvizcommand");
  semantic_map::Rvizcommand srv;
  
  if(argc == 3){
  srv.request.a = argv[1];
  srv.request.b = atoll(argv[2]);
  }
  else if(argc == 2){
    srv.request.a = argv[1];
    srv.request.b = 0;
  }
  else{
    ROS_WARN("INCORRECT INPUT");
  }
  if (client.call(srv))
  {
    ROS_INFO("%s",srv.response.response.c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service Rviz command");
    return 1;
  }

  return 0;
}