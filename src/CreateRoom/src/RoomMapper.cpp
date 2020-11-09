#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include <vector>
#include <string>
#include "CreateRoom/Rvizcommand.h"
#include <fstream>


class Room{

    private:

    float x_,y_;
    float coord[2];
    int counter = 0;
    std::string type_;

   
    public:

    float coords[2][2];

    void getType(std::string type){

        type_ = type;
    }

    void getCoords(float x, float y){

        x_ = x;
        y_ = y;
        
        coords[counter][0]=x_;
        coords[counter][1]=y_;
        counter ++;

        if(counter > 1){
            counter = 0;
        }
    }

    std::string outType(){
        return type_;
    }
};



float x,y;
std::fstream MapRoom;

geometry_msgs::PointStamped lastCoordinate;

std::vector<Room> rooms;
Room room; 

std::string slet = "delete";
std::string sletalt = "delete_all";
std::string publish = "publish";
std::string help = "help";
std::string show = "show";



void callback(const geometry_msgs::PointStamped::ConstPtr &point){
    x=point->point.x;
    y=point->point.y;
   
    ROS_INFO("coordinates: x: %f, y: %f",x,y);

    room.getCoords(x,y);

};



bool command(CreateRoom::Rvizcommand::Request  &req,
         CreateRoom::Rvizcommand::Response &res)
{   
    if(req.a == slet){
        rooms.erase(rooms.begin()+req.b);
        res.response = "Room deleted";  
    }
    else if(req.a == sletalt){
        rooms.clear();
        res.response = "Deleted all rooms";
    }
    else if(req.a == help){
        res.response = "\n delete index, to delete specific room\n delete_all, to delete all rooms \n name, to save room name \n publish, to publish rooms \n show index, to show name of room at index";
    }
    else if(req.a == publish){

        MapRoom.open ("/home/ros/catkin_ws/src/CreateRoom/map/MapRoom.xml",std::ios::out);

        for(std::vector<Room>::iterator it = rooms.begin(); it != rooms.end(); it++) { 
            
            std::string room_name = it->outType();
            ROS_INFO("%s",room_name.c_str());
            float x1 = it->coords[0][0];
            float y1 = it->coords[0][1];
            float x2 = it->coords[1][0];
            float y2 = it->coords[1][1];

            ROS_INFO("x1=%f, y1=%f, x2 = %f, y2=%f",x1,y1,x2,y2);
            std::string xstr1 = std::to_string(x1);
            std::string ystr1 = std::to_string(y1);
            std::string xstr2 = std::to_string(x2);
            std::string ystr2 = std::to_string(y2);

            std::string map_thingy = "<room name=\"" + room_name + "\">< pos x=\"" + xstr1 + "\" y=\"" + ystr1 + "\" />< pos x =\"" + xstr2 + "\" y=\"" + ystr2 + "\" />< / room > ";
            
            MapRoom << map_thingy;
            
            
        }
            MapRoom.close();
            res.response = "Map created";
    }

    else if(req.a == show){
        std::vector<Room>::iterator it = rooms.begin()+req.b;
        std::string room_name = it->outType();
        res.response = room_name;
        

    }

    else{
        room.getType(req.a);
        rooms.push_back(room);
        res.response = "Room saved";
    }
  
  
  return true;
}

int main(int argc, char **argv){
        
    ros::init(argc, argv, "rviz_listener");

    ros::NodeHandle node;
    
    
    ros::Subscriber sub = node.subscribe("clicked_point", 1, callback);
    
    ros::ServiceServer service = node.advertiseService("rvizcommand", command);

    ROS_INFO("ready to map");

    ros::Rate r(100);

    while(ros::ok()){
    
    
    ros::spin();
    r.sleep();
   
    }
    return 0;
}