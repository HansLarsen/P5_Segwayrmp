#include "ros/ros.h"
#include <ros/package.h>
#include "geometry_msgs/PointStamped.h"
#include <vector>
#include <string>
#include "create_room/Rvizcommand.h"
#include <fstream>
#include <visualization_msgs/Marker.h> 


class Room{
private:

    float x_,y_;
    float coord[2];
    
    std::string type_;

public:

    float coords[2][2];
    int counter = 0;

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

    float getx(){
        return x_;
    }

    float gety(){
        return y_;
    }

    std::string outType(){
        return type_;
    }

};

class Markers{
private:
    ros::Publisher marker_pub;
    ros::Subscriber sub;
    ros::ServiceServer service;
    Room room;
    visualization_msgs::Marker marker;
    uint32_t shape = visualization_msgs::Marker::SPHERE;

    float x,y;
    std::fstream MapRoom;

    geometry_msgs::PointStamped lastCoordinate;

    std::vector<Room> rooms;
    std::string slet = "delete";
    std::string sletalt = "delete_all";
    std::string publish = "publish";
    std::string help = "help";
    std::string show = "show";
    std::string save = "save";



public:

    Markers(ros::NodeHandle *node){
        marker_pub = node->advertise<visualization_msgs::Marker>("/visualization_marker", 10 ,this);
        service = node->advertiseService("rvizcommand",&Markers::command, this);
        sub = node->subscribe("/clicked_point", 1, &Markers::callback, this);

    }


    void callback(const geometry_msgs::PointStamped::ConstPtr &point){

        x = point->point.x;
        y = point->point.y;
   
        ROS_INFO("coordinates: x: %f, y: %f",x,y);
        if(marker_pub.getNumSubscribers() < 1){
            ROS_WARN("Add Rviz subscriber");
        }
        else{
        room.getCoords(x,y);
        createMarker();
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.id=room.counter;
        marker.ns = room.outType();
        marker_pub.publish(marker);
        }
    }

    void createMarker(){
        marker.points.clear();
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();
        marker.ns ="test"; //room.outType();
        
        marker.type = shape;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration();
    }

    bool command(create_room::Rvizcommand::Request  &req,
            create_room::Rvizcommand::Response &res){
      
       if(req.a == slet){
            
            for(int i = 0; i<4; i++){

                createMarker();
                marker.ns = rooms[req.b].outType();
                marker.action = visualization_msgs::Marker::DELETE;
                marker.id = i;
                marker_pub.publish(marker);
            }
            rooms.erase(rooms.begin()+req.b);
            res.response = "Room deleted";
        }

        else if(req.a == sletalt){
            marker.action = visualization_msgs::Marker::DELETEALL;
            marker_pub.publish(marker);
            rooms.clear();
            res.response = "Deleted all rooms";
        }
        else if(req.a == help){
            res.response = "\n delete index, to delete specific room\n delete_all, to delete all rooms \n name, to save room name \n publish, to publish rooms \n show index, to show name of room at index";
        }
        else if(req.a == publish){
            std::string pkg_path = ros::package::getPath("social_segway");

            MapRoom.open(pkg_path + "/map/rooms.xml",std::ios::out);

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

                std::string map_thingy = "<room name=\"" + room_name + "\">\n<pos x=\"" + xstr1 + "\"y=\"" + ystr1 + "\" />\n<pos x=\"" + xstr2 + "\" y=\"" + ystr2 + "\" />\n</room>\n";
                
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
        else if(req.a == save){

            float nx1,ny1,nx2,ny2;
            nx1 = room.coords[0][0];
            ny1 = room.coords[0][1];
            nx2 = room.coords[1][0];
            ny2 = room.coords[1][1];
            float square[4][2] = {{nx1,ny1},{nx1,ny2},{nx2,ny2},{nx2,ny1}};
        


            createMarker();
            marker.ns = room.outType();
            marker.pose.position.x = 0;
            marker.pose.position.y = 0;
            marker.color.r = 1.0f;
            marker.scale.x = 0.2;
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            geometry_msgs::Point p;
            p.z = 0;
            marker.id = 2;
            for(int i = 0; i<4; i++){
            
            
            p.x = square[i][0];
            p.y = square[i][1];
          
            marker.points.push_back (p);
            }

            p.x = square[0][0];
            p.y = square[0][1];
            marker.points.push_back(p);
            marker_pub.publish(marker);

            createMarker();
            marker.ns = room.outType();
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.text = room.outType();
            marker.scale.x = 0.3;
            marker.scale.y = 0.3;
            marker.scale.z = 0.4;
            marker.id = 3;
            marker.pose.position.x = (nx1+nx2+0.000001)/2;
            marker.pose.position.y = (ny1+ny2+0.000001)/2;
            marker.pose.position.z = 0.3;
            marker_pub.publish(marker);

            rooms.push_back(room);

            res.response = "Room saved";
        }

        else{
            room.getType(req.a);
            res.response = "Room " + req.a + " initialized";
        }
    
    
    return true;
    }
};


int main(int argc, char **argv){

        
    ros::init(argc, argv, "rviz_listener");
    ros::NodeHandle node;
    Markers *markerobject;
    markerobject = new Markers(&node);
    
    ros::spin();

}
