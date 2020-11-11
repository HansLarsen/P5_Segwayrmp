#include <ros/ros.h>
#include <ros/package.h>
#include "xml_lib/xml_lib.h"
#include "social_segway/Object.h"
#include "social_segway/ObjectList.h"
#include "geometry_msgs/Transform.h"
#include "std_srvs/Trigger.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include <vector>
#include <string>
#include <signal.h>
#include <cmath>

using namespace xml_lib;

#ifndef XMLCheckResult
#define XMLCheckResult(a_eResult)         \
    if (a_eResult != XML_SUCCESS)         \
    {                                     \
        printf("Error: %i\n", a_eResult); \
    }
#endif

// https://shilohjames.wordpress.com/2014/04/27/tinyxml2-tutorial/
class Semantic_data_holder
{
    struct Point
    {
        float x;
        float y;
    };
    struct RoomStruct
    {
        std::string name;
        Point points[2];
    };

    std::string file_name;
    const char *room_map_name;

    XMLDocument *doc;
    XMLNode *root;
    XMLDocument *roomDoc;
    std::vector<RoomStruct> roomVector;

    XMLElement *TransformToXML(geometry_msgs::Transform transform)
    {
        XMLElement *transformElement = doc->NewElement("Transform");
        XMLElement *rotationElement = doc->NewElement("Rotation");
        rotationElement->SetAttribute("x", transform.rotation.x);
        rotationElement->SetAttribute("y", transform.rotation.y);
        rotationElement->SetAttribute("z", transform.rotation.z);
        rotationElement->SetAttribute("w", transform.rotation.w);

        XMLElement *translationElement = doc->NewElement("Translation");
        translationElement->SetAttribute("x", transform.translation.x);
        translationElement->SetAttribute("y", transform.translation.y);
        translationElement->SetAttribute("z", transform.translation.z);

        transformElement->InsertEndChild(translationElement);
        transformElement->InsertEndChild(rotationElement);
        return transformElement;
    }

    geometry_msgs::Transform XMLElementToTransform(XMLElement *transformElement)
    {
        geometry_msgs::Transform trans;
        auto tElement = transformElement->FirstChildElement("Translation");
        trans.translation.x = tElement->DoubleAttribute("x");
        trans.translation.y = tElement->DoubleAttribute("y");
        trans.translation.z = tElement->DoubleAttribute("z");

        auto rElement = transformElement->FirstChildElement("Rotation");
        trans.rotation.x = rElement->DoubleAttribute("x");
        trans.rotation.y = rElement->DoubleAttribute("y");
        trans.rotation.z = rElement->DoubleAttribute("z");
        trans.rotation.w = rElement->DoubleAttribute("w");

        return trans;
    }

    XMLElement *getRoomElement(const char *room)
    {
        XMLElement *roomElement = root->FirstChildElement();
        while (roomElement != NULL)
        {
            const char *curRoom = roomElement->Attribute("name");
            if (!strcmp(curRoom, room))
                return roomElement;
            else
                roomElement = roomElement->NextSiblingElement();
        }
        return NULL;
    }

    social_segway::Object XMLElementToObject(XMLElement *element)
    {
        social_segway::Object object;
        object.id = element->IntAttribute("id");
        object.objectClass = element->Name();
        object.type = element->Parent()->ToElement()->Name();
        object.transform = XMLElementToTransform(element->FirstChildElement("Transform"));
        return object;
    }

    XMLElement *getFurnitureElementById(int id)
    {
        XMLElement *element;
        XMLElement *room = root->FirstChildElement();

        while (room != NULL)
        {
            XMLElement *furniture = room->FirstChildElement();
            while (furniture != NULL)
            {
                if (!strcmp(furniture->Name(), "Furniture"))
                {
                    XMLElement *items = furniture->FirstChildElement();
                    while (items != NULL)
                    {
                        if (items->IntAttribute("id") == id)
                            return furniture;
                        items = items->NextSiblingElement();
                    }
                }
                furniture = furniture->NextSiblingElement();
            }
            room = room->NextSiblingElement();
        }
        return NULL;
    }

    XMLElement *getItemElementById(int id)
    {
        XMLElement *element;
        XMLElement *room = root->FirstChildElement();

        while (room != NULL)
        {
            XMLElement *furniture = room->FirstChildElement();
            while (furniture != NULL)
            {
                if (!strcmp(furniture->Name(), "Furniture"))
                {
                    XMLElement *items = furniture->FirstChildElement();
                    while (items != NULL)
                    {
                        if (!strcmp(items->Name(), "Item"))
                        {
                            if (items->IntAttribute("id") == id)
                                return items;
                        }
                        items = items->NextSiblingElement();
                    }
                }
                else
                {
                    if (furniture->FirstChildElement()->IntAttribute("id") == id)
                        return furniture;
                }

                furniture = furniture->NextSiblingElement();
            }
            room = room->NextSiblingElement();
        }
        return NULL;
    }

    std::string getRoomByPosition(float x, float y)
    {

        for (auto room : roomVector)
        {
            bool inX = false;
            bool inY = false;
            if (room.points[0].x < room.points[1].x)
            {
                if (x > room.points[0].x && x < room.points[1].x)
                    inX = true;
            }
            else
            {
                if (x < room.points[0].x && x > room.points[1].x)
                    inX = true;
            }
            if (room.points[0].y < room.points[1].y)
            {
                if (y > room.points[0].y && y < room.points[1].y)
                    inY = true;
            }
            else
            {
                if (y < room.points[0].y && y > room.points[1].y)
                    inY = true;
            }
            if (inX && inY)
            {
                //ROS_INFO_STREAM("room.name.c_str(): " << room.name);
                return room.name;
            }
        }
        return "Unknown";
    }

public:
    void resetMap()
    {
        doc->Clear();
        root = doc->NewElement("map");
        doc->InsertFirstChild(root);
        roomVector.clear();
        addRoom("Unknown");

        //Read room document:
        xml_lib::XMLError error = roomDoc->LoadFile(room_map_name);
        if (error != XML_SUCCESS)
        {
            ROS_ERROR_STREAM("FAILED TO OPEN " << room_map_name <<  " KILLING SEMANTIC NODE");
            exit(-1);
        }
        auto room = roomDoc->FirstChildElement();
        while (room != NULL)
        {
            RoomStruct roomStr;
            roomStr.name = room->Attribute("name");
            //ROS_INFO_STREAM("roomstr.name: " << roomStr.name);

            auto posElement = room->FirstChildElement();
            for (int i = 0; i < 2; i++)
            {
                Point point;
                point.x = posElement->FloatAttribute("x");
                point.y = posElement->FloatAttribute("y");
                roomStr.points[i] = point;
                posElement = posElement->NextSiblingElement();
            }

            addRoom(room->Attribute("name"));
            room = room->NextSiblingElement();
            roomVector.push_back(roomStr);
        }

        //for (auto a : roomVector)
        //    ROS_INFO_STREAM("roomVec: " << a.points[0].x << ", " << a.points[0].y << ", " << a.points[1].x << ", " << a.points[1].y );

    }
    
    Semantic_data_holder(ros::NodeHandle *nh, std::string map_file_name, bool ERASE_OLD_MAP)
    {
        std::string pkg_path = ros::package::getPath("social_segway");
        std::string map_path = pkg_path + "/map/" + map_file_name;
        std::string room_path = pkg_path + "/map/rooms.xml";

        file_name = map_path;
        room_map_name = room_path.c_str();
        //ROS_INFO_STREAM("file_name: " << file_name);
        //ROS_INFO_STREAM("room_map_name: " << room_map_name);

        //ROS_INFO_STREAM("Creating map in: ");
        //ROS_INFO_STREAM(file_name);
        doc = new XMLDocument();
        roomDoc = new XMLDocument();

        XMLError error = doc->LoadFile(file_name.c_str());
        if (error == XML_SUCCESS && ERASE_OLD_MAP)
        {
            //delete old map
            doc->Clear();
        }
        resetMap();
    }

    void addRoom(const char *room)
    {
        XMLElement *element = doc->NewElement("Room");
        element->SetAttribute("name", room);
        root->InsertEndChild(element);
    }

    bool addObjectToRoom(const char *room, social_segway::Object object)
    {
        //find room by looping through all rooms:
        auto roomElement = getRoomElement(room);
        if (roomElement == NULL)
        {
            ROS_WARN_STREAM("Trying to add object: '" + object.objectClass + "' to room: '" + std::string(room) + "' which does not exist!");
            return false;
        }

        XMLElement *objectType = doc->NewElement(object.type.c_str());
        XMLElement *objectClass = doc->NewElement(object.objectClass.c_str());
        objectClass->SetAttribute("id", object.id);
        XMLElement *transform = TransformToXML(object.transform);
        objectClass->InsertEndChild(transform);
        objectType->InsertEndChild(objectClass);
        roomElement->InsertEndChild(objectType);
        return true;
    }

    bool addObjectByPosition(social_segway::Object object)
    {
        auto room = getRoomByPosition(object.transform.translation.x, object.transform.translation.y);
        //ROS_INFO_STREAM("addObjectByPos room = " << room);
        return addObjectToRoom(room.c_str(), object);
    }

    bool placeObjectOnFurniture(social_segway::Object item, social_segway::Object furniture)
    {
        return placeObjectOnFurnitureById(item.id, furniture.id);
    }

    bool placeObjectOnFurnitureById(int itemId, int furnitureId)
    {
        XMLElement *itemEle = getItemElementById(itemId);
        XMLElement *furnitureEle = getFurnitureElementById(furnitureId);
        if (itemEle == NULL || furnitureEle == NULL)
            return false;

        std::cout << "insert: " << (bool)furnitureEle->InsertEndChild(itemEle) << "\n\n";
        return true;
    }

    std::vector<const char *> getRooms()
    {
        XMLElement *room = root->FirstChildElement();
        std::vector<const char *> rooms;
        while (room != NULL)
        {
            rooms.push_back(room->Attribute("room"));
            room = room->NextSiblingElement();
        }
        return rooms;
    }

    std::vector<const char *> getObjectsInRoom(const char *room)
    {
        XMLElement *roomElement = getRoomElement(room);
        XMLElement *object = roomElement->FirstChildElement();
        std::vector<const char *> objects;
        while (object != NULL)
        {
            objects.push_back(object->Name());
            object = object->NextSiblingElement();
        }
        return objects;
    }

    std::vector<social_segway::Object> getLooseObjects()
    {
        std::vector<social_segway::Object> objects;
        XMLElement *room = root->FirstChildElement();

        while (room != NULL)
        {
            XMLElement *furniture = room->FirstChildElement();

            while (furniture != NULL)
            {
                if (strcmp(furniture->Name(), "Furniture")) // any non-furniture
                    objects.push_back(XMLElementToObject(furniture->FirstChildElement()));

                furniture = furniture->NextSiblingElement();
            }
            room = room->NextSiblingElement();
        }
        return objects;
    }

    std::vector<social_segway::Object> getAllFurniture()
    {
        std::vector<social_segway::Object> furnitures;
        XMLElement *room = root->FirstChildElement();

        while (room != NULL)
        {
            XMLElement *furniture = room->FirstChildElement();
            while (furniture != NULL)
            {
                if (!strcmp(furniture->Name(), "Furniture"))
                {
                    XMLElement *items = furniture->FirstChildElement();
                    while (items != NULL)
                    {
                        if (strcmp(items->Name(), "Item"))
                            furnitures.push_back(XMLElementToObject(items));
                        items = items->NextSiblingElement();
                    }
                }
                furniture = furniture->NextSiblingElement();
            }
            room = room->NextSiblingElement();
        }

        return furnitures;
    }

    std::vector<social_segway::Object> getAllObjects()
    {
        std::vector<social_segway::Object> objects;
        XMLElement *room = root->FirstChildElement();

        while (room != NULL)
        {
            XMLElement *furniture = room->FirstChildElement();

            while (furniture != NULL)
            {
                if (!strcmp(furniture->Name(), "Furniture"))
                {
                    XMLElement *items = furniture->FirstChildElement();
                    while (items != NULL)
                    {
                        if (strcmp(items->Name(), "Item"))
                            objects.push_back(XMLElementToObject(items));
                        else
                            objects.push_back(XMLElementToObject(items->FirstChildElement()));

                        items = items->NextSiblingElement();
                    }
                }
                else //loose object
                    objects.push_back(XMLElementToObject(furniture->FirstChildElement()));

                furniture = furniture->NextSiblingElement();
            }
            room = room->NextSiblingElement();
        }
        return objects;
    }

    void saveMap()
    {
        XMLError error = doc->SaveFile(file_name.c_str());
        XMLCheckResult(error);
        ROS_INFO_STREAM("Saved semantic map at: " << file_name.c_str());
    }
};

void testSemanticDataHolderClass(ros::NodeHandle *nh)
{
    Semantic_data_holder *map;
    map = new Semantic_data_holder(nh, "test_map.xml", true);
    map->addRoom("Dining room");
    map->addRoom("Bedroom");
    map->addRoom("Living room");

    //ObjectStruct object;
    social_segway::Object object;
    object.objectClass = "Chair";
    object.type = "Furniture";
    object.id = 1;
    geometry_msgs::Transform trans;
    trans.translation.x = 1.2;
    trans.translation.y = 2.3;
    trans.translation.z = 3.4;

    trans.rotation.x = 1.5;
    trans.rotation.y = 2.6;
    trans.rotation.z = 3.7;
    trans.rotation.w = 4.8;
    object.transform = trans;

    map->addObjectToRoom("Dining room", object);
    object.id = 2;
    object.objectClass = "Sofa";
    map->addObjectToRoom("Dining room", object);
    object.id = 3;
    object.objectClass = "Bottle";
    object.type = "Item";
    map->addObjectToRoom("Dining room", object);
    object.id = 4;
    object.objectClass = "Laptop";
    map->addObjectToRoom("Dining room", object);

    auto objects = map->getObjectsInRoom("Dining room");
    std::cout << objects.size() << std::endl;
    for (auto object : objects)
    {
        std::cout << object << ", ";
    }
    std::cout << std::endl
              << "\n\n\n";

    auto objectEles = map->getAllObjects();
    std::cout << "Total object count: " << objectEles.size() << "\n";
    for (auto object : objectEles)
        std::cout << "Object: " << object.id << ", " << object.type << ", " << object.objectClass << ", "
                  << object.transform.translation.x << ", " << object.transform.rotation.x << "\n";

    objectEles = map->getAllFurniture();

    std::cout << "Total furniture count: " << objectEles.size() << "\n";
    for (auto object : objectEles)
        std::cout << "Furniture: " << object.objectClass << ", id: " << object.id << "\n";

    objectEles = map->getLooseObjects();
    std::cout << "Loose objects: " << objectEles.size() << "\n";
    for (auto object : objectEles)
        std::cout << "Item: " << object.objectClass << ", id: " << object.id << "\n";

    std::cout << "Placeobjectonfurniturebyid(4,2): " << map->placeObjectOnFurnitureById(4, 2) << "\n";

    map->saveMap();
}

class Semantic_node
{
    bool compare;
    float allowedDeviation = 5; // idk just some value for now
    float allowedDeviation2;
    int idCounter;
    double distance;
    float x1, y1, z1, x2, y2, z2;

    ros::Subscriber changedDetectionSub;
    ros::Subscriber detectionSub;
    
    ros::Publisher markerPub;
    ros::Timer timer;
    Semantic_data_holder *map;
    Semantic_data_holder *old_map;
    ros::ServiceServer service_save_map;
    ros::ServiceServer service_reset_map;

    void detectionCallback(const social_segway::ObjectList &data)
    {
        for (auto detectedObject : data.objects)
        {
            //ROS_INFO_STREAM("for loop");
            auto allObjects = map->getAllObjects();
            //ROS_INFO_STREAM("allObjects.size(): " << allObjects.size());
            if (allObjects.size() == 0) // no objects yet, add first:
            {
                //ROS_INFO_STREAM("no objects yet");
                // add item to list
                detectedObject.id = idCounter;
                idCounter++;
                map->addObjectByPosition(detectedObject);
                continue;
            }
            x1 = detectedObject.transform.translation.x;
            y1 = detectedObject.transform.translation.y;
            z1 = detectedObject.transform.translation.z;

            bool merged = false;
            for (auto object : allObjects)
            {
                x2 = object.transform.translation.x;
                y2 = object.transform.translation.y;
                z2 = object.transform.translation.z;

                distance = std::hypot(std::hypot(x1 - x2, y1 - y2), z1 - z2);

                if (distance < allowedDeviation && detectedObject.objectClass == object.objectClass && detectedObject.type == object.type)
                {
                    ROS_INFO_STREAM("Merging item: " << detectedObject.objectClass);
                    merged = true;
                    // Merge items: detectedObject and object (or ignore detectedObject?)
                    //ROS_INFO_STREAM("MERGING");

                    //mergeObjects(detectedObject, object); //Ignored for now, uncomment once implemented!
                    break; //
                }
            }
            if (!merged)
            {
                // add item to list
                ROS_INFO_STREAM("Adding new item: " << detectedObject.objectClass);
                detectedObject.id = idCounter;
                idCounter++;
                map->addObjectByPosition(detectedObject);
            }
        }
    }

    void checkOnTopAll()
    {
        // Check if LooseObject is on top of Furniture object
        auto allFurniture = map->getAllFurniture();
        //ROS_INFO_STREAM("Checking On Top, allFurniture.size(): " << allFurniture.size());

        for (auto Furniture : allFurniture)
        {
            x1 = Furniture.transform.translation.x;
            y1 = Furniture.transform.translation.y;
            z1 = Furniture.transform.translation.z;

            auto allLooseObjects = map->getLooseObjects();
            //ROS_INFO_STREAM("#loose objecst: " << allLooseObjects.size());
            for (auto LooseObject : allLooseObjects)
            {
                x2 = LooseObject.transform.translation.x;
                y2 = LooseObject.transform.translation.y;
                z2 = LooseObject.transform.translation.z;

                distance = std::hypot(x1 - x2, y1 - y2);

                if (z2 > z1)
                { // if higher
                    //ROS_INFO_STREAM("Checking On Top, object is higher!");

                    if (Furniture.objectClass == "dinner_table")
                        allowedDeviation2 = 1; // estimated radius of the Furniture upper surface
                    else if (Furniture.objectClass == "shelve")
                        allowedDeviation2 = 2;
                    else if (Furniture.objectClass == "coffee_table")
                        allowedDeviation2 = 0.5;
                    else
                        allowedDeviation2 = 0.2;
                    //ect

                    if (distance < allowedDeviation2)
                    {
                        //ROS_INFO_STREAM("placeobjetonfurin");
                        if (map->placeObjectOnFurniture(LooseObject, Furniture))
                            ROS_INFO_STREAM("Placed on top success");
                        else
                            ROS_ERROR_STREAM("failed to place item on furniture");
                    }
                }
            }
        }
    }

    void publishMarkers()
    {
        auto objects = map->getAllObjects();
        std::vector<visualization_msgs::Marker> markerArray;
        for (int i = 0; i < objects.size(); i++)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time();
            marker.ns = "semantic_map";
            marker.id = i;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = objects[i].transform.translation.x;
            marker.pose.position.y = objects[i].transform.translation.y;
            marker.pose.position.z = objects[i].transform.translation.z;
            marker.pose.orientation.x = objects[i].transform.rotation.x;
            marker.pose.orientation.y = objects[i].transform.rotation.y;
            marker.pose.orientation.z = objects[i].transform.rotation.z;
            marker.pose.orientation.w = objects[i].transform.rotation.w;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.a = 1.0; // Don't forget to set the alpha!
            if (objects[i].type == "Item")
            {
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
            }
            else
            {
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
            }

            marker.lifetime = ros::Duration(1.1);
            markerArray.push_back(marker);
        }

        visualization_msgs::MarkerArray msg;
        msg.markers = markerArray;
        markerPub.publish(msg);
    }

    void OneSecTimerCallback(const ros::TimerEvent &)
    {
        checkOnTopAll();
        publishMarkers();
    }

    bool mergeObjects(social_segway::Object detectedObject, social_segway::Object object)
    {
        //Merge somehow
    }

    bool saveMap_callback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
    {
        map->saveMap();
        response.success = true;
        response.message = "Saved map";
        return true;
    }
    bool resetMap_callback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
    {
        map->resetMap();
        response.success = true;
        response.message = "Reset map";
        return true;
    }

    void changeDetectionCallback(const social_segway::ObjectList &data)
    {        
        for (auto detectedObject : data.objects)
        {
            x1 = detectedObject.transform.translation.x;
            y1 = detectedObject.transform.translation.y;
            z1 = detectedObject.transform.translation.z;

            bool merged = false;

            auto allChangedObjects = map->getAllObjects();
            for (auto changedObject : allChangedObjects) // check if merge
            {
                if (allChangedObjects.size() == 0) // no objects in map to merge
                    break;
                
                x2 = changedObject.transform.translation.x;
                y2 = changedObject.transform.translation.y;
                z2 = changedObject.transform.translation.z;

                distance = std::hypot(std::hypot(x1 - x2, y1 - y2), z1 - z2);

                if (distance < allowedDeviation && detectedObject.objectClass == changedObject.objectClass && detectedObject.type == changedObject.type)
                { 
                    ROS_INFO_STREAM("Merging item: " << detectedObject.objectClass);
                    merged = true;
                    //mergeObjects(detectedObject, changedObject); //Ignored for now, uncomment once implemented!
                    break;
                }
            }

            if (!merged)
            {
                auto allObjects = old_map->getAllObjects();

                if (allObjects.size() == 0) // no objects in map
                {
                    ROS_FATAL("NO OBJECTS IN MAP.XML");
                    exit(1);
                }

                for (auto object : allObjects)
                {
                    x2 = object.transform.translation.x;
                    y2 = object.transform.translation.y;
                    z2 = object.transform.translation.z;

                    distance = std::hypot(std::hypot(x1 - x2, y1 - y2), z1 - z2);

                    if (distance > allowedDeviation && detectedObject.objectClass == object.objectClass && detectedObject.type == object.type)
                    { 
                        // add item to list
                        ROS_INFO_STREAM("Adding changed item: " << detectedObject.objectClass);
                        map->addObjectByPosition(object);
                        break;
                    }
                }
            }
        }
    }

public:
    Semantic_node(ros::NodeHandle *nh)
    {
        ros::param::get("compare", compare);

        if (compare)
        {
            map = new Semantic_data_holder(nh, "map_changes.xml", true);
            old_map = new Semantic_data_holder(nh, "map.xml", false);
            changedDetectionSub = nh->subscribe("detected_objects", 1000, &Semantic_node::changeDetectionCallback, this);
            ROS_INFO_STREAM("param mode: compare");
        }
        else
        {
            map = new Semantic_data_holder(nh, "map.xml", true);
            detectionSub = nh->subscribe("detected_objects", 1000, &Semantic_node::detectionCallback, this);
            ROS_INFO_STREAM("param mode: original");
        }

        markerPub = nh->advertise<visualization_msgs::MarkerArray>("map_markers", 10);

        service_save_map = nh->advertiseService("save_map", &Semantic_node::saveMap_callback, this);
        service_reset_map = nh->advertiseService("reset_map", &Semantic_node::resetMap_callback, this);

        timer = nh->createTimer(ros::Duration(1.0), &Semantic_node::OneSecTimerCallback, this);
        idCounter = 1;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "semantic_node");
    ros::NodeHandle n;
    Semantic_node *node;
    node = new Semantic_node(&n);
    ros::spin();

    return 0;
}