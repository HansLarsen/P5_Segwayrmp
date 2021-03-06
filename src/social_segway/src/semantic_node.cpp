#include <ros/ros.h>
#include <ros/package.h>
#include "xml_lib/xml_lib.h"
#include "cameralauncher/Object.h"
#include "cameralauncher/ObjectList.h"
#include "social_segway/GetObjectsInRoom.h"
#include "social_segway/GetRooms.h"
#include "social_segway/CheckObject.h"
#include "social_segway/GetObjectPosById.h"
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
    std::string room_map_name;

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

    cameralauncher::Object XMLElementToObject(XMLElement *element)
    {
        cameralauncher::Object object;
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
    void readyRoomVector()
    {
        //Read room document:
        xml_lib::XMLError error = roomDoc->LoadFile(room_map_name.c_str());
        if (error != XML_SUCCESS)
        {
            ROS_ERROR_STREAM("FAILED TO OPEN ROOMS file: " << room_map_name << " KILLING SEMANTIC NODE");
            exit(-1);
        }

        roomVector.clear();
        auto room = roomDoc->FirstChildElement();
        while (room != NULL)
        {
            RoomStruct roomStr;
            roomStr.name = room->Attribute("name");
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
        //  ROS_INFO_STREAM("roomVec: " << a.points[0].x << ", " << a.points[0].y << ", " << a.points[1].x << ", " << a.points[1].y);
    }

    bool removeElement(XMLElement *ele)
    {
        if (ele == NULL)
            return false;
        auto parent = ele->Parent();
        if (parent == NULL)
            return false;
        parent->DeleteChild(ele);
        return true;
    }

public:
    bool removeItemById(int id)
    {
        auto ele = getItemElementById(id);
        return removeElement(ele);
    }

    bool removeFurnitureById(int id)
    {
        auto furnitureElement = getFurnitureElementById(id);
        auto roomElement = furnitureElement->Parent();
        XMLElement *itemElement = furnitureElement->FirstChildElement();

        while (itemElement != NULL)
        {
            if (!strcmp(itemElement->Name(), "Item"))
            {
                //ROS_INFO("MOVING ITEM");
                roomElement->InsertEndChild(itemElement);
            }
            itemElement = itemElement->NextSiblingElement();
        }

        //ROS_INFO_STREAM("Dat element: " << furnitureElement);
        return removeElement(furnitureElement);
    }

    void resetMap()
    {
        doc->Clear();
        root = doc->NewElement("map");
        doc->InsertFirstChild(root);
        addRoom("Unknown");
    }

    Semantic_data_holder(ros::NodeHandle *nh, std::string map_file_name, bool ERASE_OLD_MAP)
    {
        std::string pkg_path = ros::package::getPath("social_segway");
        std::string map_path = pkg_path + "/map/" + map_file_name;
        std::string room_path = pkg_path + "/map/rooms.xml";

        file_name = map_path;
        room_map_name = room_path;
        //ROS_INFO_STREAM("file_name: " << file_name);
        //ROS_INFO_STREAM("room_map_name: " << room_map_name);

        //ROS_INFO_STREAM("Creating map in: ");
        //ROS_INFO_STREAM(file_name);
        doc = new XMLDocument();
        roomDoc = new XMLDocument();
        if (ERASE_OLD_MAP)
        {
            //delete old map
            resetMap();
            readyRoomVector();
        }
        else if (!ERASE_OLD_MAP)
        {
            XMLError error = doc->LoadFile(file_name.c_str());
            if (error != XML_SUCCESS)
            {
                ROS_ERROR_STREAM("Failed to load map: " << file_name << ", KILLING SEMANTIC NODE");
                exit(-1);
            }

            root = doc->FirstChildElement();
            readyRoomVector();
        }
    }

    void addRoom(const char *room)
    {
        XMLElement *element = doc->NewElement("Room");
        element->SetAttribute("name", room);
        root->InsertEndChild(element);
    }

    bool addObjectToRoom(const char *room, cameralauncher::Object object)
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

    bool addObjectByPosition(cameralauncher::Object object)
    {
        auto room = getRoomByPosition(object.transform.translation.x, object.transform.translation.y);
        //ROS_INFO_STREAM("addObjectByPos room = " << room);
        return addObjectToRoom(room.c_str(), object);
    }

    bool placeObjectOnFurniture(cameralauncher::Object item, cameralauncher::Object furniture)
    {
        return placeObjectOnFurnitureById(item.id, furniture.id);
    }

    bool placeObjectOnFurnitureById(int itemId, int furnitureId)
    {
        XMLElement *itemEle = getItemElementById(itemId);
        XMLElement *furnitureEle = getFurnitureElementById(furnitureId);
        if (itemEle == NULL || furnitureEle == NULL)
            return false;

        (bool)furnitureEle->InsertEndChild(itemEle);
        return true;
    }

    std::vector<std::string> getRooms()
    {
        XMLElement *room = root->FirstChildElement();
        std::vector<std::string> rooms;
        while (room != NULL)
        {
            rooms.push_back(std::string(room->Attribute("name")));
            room = room->NextSiblingElement();
        }
        return rooms;
    }

    std::vector<std::string> getObjectNamesInRoom(const char *room)
    {
        XMLElement *roomElement = getRoomElement(room);
        XMLElement *object = roomElement->FirstChildElement();
        std::vector<std::string> objects;
        while (object != NULL)
        {
            objects.push_back(std::string(object->Name()));
            object = object->NextSiblingElement();
        }
        return objects;
    }

    std::vector<cameralauncher::Object> getLooseObjects()
    {
        std::vector<cameralauncher::Object> objects;
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

    std::vector<cameralauncher::Object> getAllFurniture()
    {
        std::vector<cameralauncher::Object> furnitures;
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

    std::vector<cameralauncher::Object> getAllObjects()
    {
        std::vector<cameralauncher::Object> objects;
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

    std::vector<cameralauncher::Object> getObjectsInRoom(const char *room)
    {
        std::vector<cameralauncher::Object> objects;
        XMLElement *roomElement = getRoomElement(room);
        if (roomElement == NULL)
            return objects;

        XMLElement *furniture = roomElement->FirstChildElement();

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
        return objects;
    }

    void saveMap()
    {
        XMLError error = doc->SaveFile(file_name.c_str());
        XMLCheckResult(error);
        //ROS_INFO_STREAM("Saved semantic map at: " << file_name.c_str());
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
    cameralauncher::Object object;
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

    map->removeItemById(4);
    map->removeFurnitureById(1);
    return;
    auto objects = map->getObjectNamesInRoom("Dining room");
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
    std::vector<int> timesFound;
    std::vector<ros::Time> detectedTimeStamp;
    bool compare;
    float allowedDeviation = 0.5; // merge deviation
    float allowedDeviation2 = 1;  // place on top deviation
    int idCounter;
    double distance;
    float x1, y1, z1, x2, y2, z2;

    ros::Subscriber changedDetectionSub;
    ros::Subscriber detectionSub;

    ros::Publisher markerPub;
    ros::Timer timer;
    Semantic_data_holder *map;
    Semantic_data_holder *changes_map;
    ros::ServiceServer service_save_map;
    ros::ServiceServer service_reset_map;
    ros::ServiceServer service_checkObject;
    ros::ServiceServer getRoomsSrv;
    ros::ServiceServer getObjectsInRoomSrv;
    ros::ServiceServer getLocationById;

    void detectionCallback(const cameralauncher::ObjectList &data)
    {
        for (auto detectedObject : data.objects)
        {
            if (detectedObject.type == "furniture")
                detectedObject.type = "Furniture";
            if (detectedObject.type == "item")
                detectedObject.type = "Item";

            //ROS_INFO_STREAM("for loop");
            auto allObjects = map->getAllObjects();
            ROS_INFO_STREAM("Detected object: " << detectedObject.objectClass);

            /*for (auto object : allObjects)
            {
                ROS_INFO_STREAM("debug1: Object: " << object.objectClass);
            }*/

            if (allObjects.size() == 0) // no objects yet, add first:
            {
                //ROS_INFO_STREAM("no objects yet");
                // add item to list
                ROS_INFO_STREAM("Adding new item: " << detectedObject.objectClass);
                detectedObject.id = idCounter;
                idCounter++;
                timesFound.push_back(1);
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

                //ROS_INFO_STREAM("debug2: Objectd: " << object.objectClass << ", Distance: " << distance);
                if (distance < allowedDeviation && detectedObject.objectClass == object.objectClass && detectedObject.type == object.type)
                {
                    ROS_INFO_STREAM("Merging item: " << detectedObject.objectClass);
                    merged = true;
                    if (!mergeObjects(detectedObject, object))
                    {
                    }
                    //ROS_warn_STREAM("Failed to merged item: " << detectedObject.objectClass);
                    break; //
                }
            }
            if (!merged)
            {
                // add item to list
                ROS_INFO_STREAM("Adding new item: " << detectedObject.objectClass);
                detectedObject.id = idCounter;
                idCounter++;
                timesFound.push_back(1);
                map->addObjectByPosition(detectedObject);
            }
        }
    }

    void changeDetectionCallback(const cameralauncher::ObjectList &data)
    {
        CheckTimeNowReady();

        for (auto detectedObject : data.objects)
        {

            if (detectedObject.objectClass == "diningtable") // ignore tables in phase 1 for testing purposes
                continue;

            x1 = detectedObject.transform.translation.x;
            y1 = detectedObject.transform.translation.y;
            z1 = detectedObject.transform.translation.z;

            bool merged = false;

            auto allChangedObjects = changes_map->getAllObjects();
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
                    ROS_INFO_STREAM("[semantic node] object detected: " << detectedObject.objectClass);
                    //ROS_INFO_STREAM("Cakes " << detectedObject.id);
                    detectedTimeStamp.at(changedObject.id) = ros::Time::now();

                    ROS_INFO_STREAM("Merging item: " << detectedObject.objectClass);
                    merged = true;
                    if (!mergeObjects(detectedObject, changedObject))
                        ROS_WARN_STREAM("Failed to merged item: " << detectedObject.objectClass);
                    break;
                }
                else if (distance > allowedDeviation && detectedObject.objectClass == changedObject.objectClass && detectedObject.type == changedObject.type)
                {
                    merged = true;
                    detectedObject.id = changedObject.id;
                    detectedTimeStamp.at(detectedObject.id) = ros::Time::now();
                    if (detectedObject.type == "Item")
                    {
                        if (!changes_map->removeItemById(changedObject.id))
                            ROS_WARN_STREAM("[semantic node] Found duplicate item but failed to replace: " << detectedObject.objectClass);
                        else
                        {
                            ROS_WARN_STREAM("[semantic node] Found duplicate item and replaced: " << detectedObject.objectClass);
                            // add item to list
                            changes_map->addObjectByPosition(detectedObject);
                        }
                    }
                    else if (detectedObject.type == "Furniture")
                    {
                        if (!changes_map->removeFurnitureById(changedObject.id))
                            ROS_WARN_STREAM("[semantic node] Found duplicate furniture but failed to replace: " << detectedObject.objectClass);
                        else
                        {
                            ROS_WARN_STREAM("[semantic node] Found duplicate furniture and replaced: " << detectedObject.objectClass);
                            // add item to list
                            changes_map->addObjectByPosition(detectedObject);
                        }
                    }
                    else
                        ROS_WARN_STREAM("[semantic node] Found duplicate unitentified type object: " << detectedObject.objectClass);
                    break;
                }
            }

            if (!merged)
            {
                ROS_INFO_STREAM("[semantic node] object: " << detectedObject.objectClass << " not merged");
                auto allObjects = map->getAllObjects();

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
                        detectedObject.id = object.id;
                        changes_map->addObjectByPosition(detectedObject);
                        detectedTimeStamp.at(object.id) = ros::Time::now();
                        break;
                    }
                    else if (distance < allowedDeviation && detectedObject.objectClass == object.objectClass && detectedObject.type == object.type)
                    {
                        detectedTimeStamp.at(object.id) = ros::Time::now();
                        ROS_INFO_STREAM("Found but not adding changed item: " << detectedObject.objectClass);
                        break;
                    }
                }
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
                    if (distance < allowedDeviation2)
                    {
                        //ROS_INFO_STREAM("placeobjetonfurin");
                        if (map->placeObjectOnFurniture(LooseObject, Furniture))
                            ROS_INFO_STREAM("Placed " << LooseObject.objectClass << " on " << Furniture.objectClass);
                        else
                            ROS_ERROR_STREAM("Failed to place" << LooseObject.objectClass << " on " << LooseObject.objectClass);
                    }
                }
            }
        }
    }

    void removeNotChanged()
    {
        auto allChangedObjects = changes_map->getAllObjects();
        for (auto changedObject : allChangedObjects) // check if merge
        {
            x1 = changedObject.transform.translation.x;
            y1 = changedObject.transform.translation.y;
            z1 = changedObject.transform.translation.z;
            
            auto allObjects = map->getAllObjects();
            for (auto object : allObjects)
            {
                x2 = object.transform.translation.x;
                y2 = object.transform.translation.y;
                z2 = object.transform.translation.z;
                distance = std::hypot(std::hypot(x1 - x2, y1 - y2), z1 - z2);
                if (distance < allowedDeviation && changedObject.id == object.id)
                {
                    if (changedObject.type == "Item")
                    {
                        if (!changes_map->removeItemById(changedObject.id))
                            ROS_INFO_STREAM("Found not-changed item but failed to remove: " << changedObject.objectClass);
                        else
                            ROS_INFO_STREAM("Found not-changed item and removed: " << changedObject.objectClass);
                    }
                    else if (changedObject.type == "Furniture")
                    {
                        if (!changes_map->removeFurnitureById(changedObject.id))
                            ROS_INFO_STREAM("Found not-changed furniture but failed to remove: " << changedObject.objectClass);
                        else
                            ROS_INFO_STREAM("Found not-changed furniture and removed: " << changedObject.objectClass);
                    }
                    else
                        ROS_INFO_STREAM("Found not-changed unitentified type object: " << changedObject.objectClass);
                    break;
                }
            }
        }
    }

    void publishMarkers()
    {
        auto objects = map->getAllObjects();
        std::vector<visualization_msgs::Marker> markerArray;
        int j = objects.size();
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

            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.id = j;
            j++;
            marker.scale.z = 0.2;
            marker.pose.position.z += 0.25;
            marker.text = objects[i].objectClass;
            marker.color.r = 1.0;
            marker.color.b = 1.0;
            marker.color.g = 1.0;
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

        if (compare)
        {
            changes_map->saveMap();
            removeNotChanged();
        }
        else
            map->saveMap();
    }

    bool mergeObjects(cameralauncher::Object newObject, cameralauncher::Object oldObject)
    { // the more times the object have been found the less importance will the new point have when merging
        // this is so we dont just take the middle point everytime and so we dont have to save all points for k-means clustering or so
        //ROS_INFO_STREAM("size of times found at mearge:" << timesFound.size());
        timesFound.at(oldObject.id)++;
        int tf;
        tf = timesFound.at(oldObject.id);
        x1 = newObject.transform.translation.x;
        y1 = newObject.transform.translation.y;
        z1 = newObject.transform.translation.z;
        x2 = oldObject.transform.translation.x;
        y2 = oldObject.transform.translation.y;
        z2 = oldObject.transform.translation.z;

        oldObject.transform.translation.x = ((x2 * (tf - 1)) + x1) / tf;
        oldObject.transform.translation.y = ((y2 * (tf - 1)) + y1) / tf;
        oldObject.transform.translation.z = ((z2 * (tf - 1)) + z1) / tf;
        //ROS_INFO_STREAM("Object: " << oldObject.objectClass << ", timesFound: " << tf << ", new x coordinate: " << oldObject.transform.translation.x);

        if (compare)
        {
            if (oldObject.type == "Furniture")
            {
                //changes_map->removeFurnitureById(oldObject.id);
                if (!changes_map->removeFurnitureById(oldObject.id))
                {
                    ROS_WARN_STREAM("Merging FAILED: " << oldObject.objectClass);
                    return false;
                }
            }
            else if (oldObject.type == "Item")
            {
                //changes_map->removeItemById(oldObject.id);
                if (!changes_map->removeItemById(oldObject.id))
                {
                    ROS_WARN_STREAM("Merging FAILED: " << oldObject.objectClass);
                    return false;
                }
            }
            else
            {
                ROS_WARN_STREAM("Asked to remove invalid object type: " << oldObject.type << ", class: " << oldObject.objectClass);
                return false;
            }
            changes_map->addObjectByPosition(oldObject);
        }
        else
        {
            if (oldObject.type == "Furniture")
            {
                //map->removeFurnitureById(oldObject.id);
                //std::cout << " removeFurnitureById: "<< map->removeFurnitureById(oldObject.id) << ", Furniture:"<< oldObject.objectClass << std::endl;
                if (!map->removeFurnitureById(oldObject.id))
                {
                    ROS_WARN_STREAM("Merging FAILED: " << oldObject.objectClass);
                    return false;
                }
            }
            else if (oldObject.type == "Item")
            {
                //map->removeItemById(oldObject.id);
                //std::cout << " removeItemById: "<< map->removeItemById(oldObject.id) << ", Item:"<< oldObject.objectClass << std::endl;
                if (!map->removeItemById(oldObject.id))
                {
                    ROS_WARN_STREAM("Merging FAILED: " << oldObject.objectClass);
                    return false;
                }
            }
            else
            {
                ROS_WARN_STREAM("Asked to remove invalid object type: " << oldObject.type << ", class: " << oldObject.objectClass);
                return false;
            }
            map->addObjectByPosition(oldObject);
        }
        return true;
    }

    bool saveMap_callback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
    {
        if (compare)
        {
            changes_map->saveMap();
        }
        else
        {
            map->saveMap();
        }
        response.success = true;
        response.message = "Saved map";
        return true;
    }

    bool resetMap_callback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
    {
        if (compare)
        {
            changes_map->resetMap();
        }
        else
        {
            map->resetMap();
        }
        response.success = true;
        response.message = "Reset map";
        ROS_INFO_STREAM("Reset semantic map");
        return true;
    }

    void CheckTimeNowReady()
    {
        while (true) //ros.wiki says to do this before using ros::time::now()
        {
            if (ros::Time::now() != ros::Time(0))
                break;
            ROS_WARN_STREAM("ros::Time::now() is not ready");
            ros::Duration(0.5).sleep();
        }
    }

    bool checkObject_callback(social_segway::CheckObject::Request &request, social_segway::CheckObject::Response &response)
    {
        CheckTimeNowReady();

        if (request.id > detectedTimeStamp.size())
        {
            ROS_WARN_STREAM("requested time for ID: " << request.id << " not in array");
            response.success = false;
        }
        else if (ros::Time::now().toSec() - detectedTimeStamp.at(request.id).toSec() < 10)
        {
            response.success = true;
        }
        return true;
    }

    bool getLocationById_callback(social_segway::GetObjectPosById::Request &request, social_segway::GetObjectPosById::Response &response)
    {
        if (!compare && request.getChangedMap == true)
        {
            ROS_WARN_STREAM("Cannot call from changed map when not in compare mode!");
            return false;
        }
        if (request.getChangedMap)
        {
            auto allChangedObjects = changes_map->getAllObjects();
            for (auto changedObject : allChangedObjects)
            {
                if (request.id == changedObject.id)
                {
                    response.translation = changedObject.transform.translation;
                    response.success = true;
                    return true;
                }
            }
            response.success = false;
            return true;
        }
        else if (!request.getChangedMap)
        {
            auto allObjects = map->getAllObjects();
            for (auto object : allObjects)
            {
                if (request.id == object.id)
                {
                    response.translation = object.transform.translation;
                    response.success = true;
                    return true;
                }
            }
            response.success = false;
            return true;
        }
    }

    bool getRooms_callback(social_segway::GetRooms::Request &request, social_segway::GetRooms::Response &response)
    {
        std::vector<std::string> rooms;
        rooms = map->getRooms();
        for (int i = 0; i < rooms.size(); i++)
        {
            if (rooms[i] == "Unknown")
                rooms.erase(rooms.begin() + i);
        }
        response.rooms = rooms;
        return true;
    }

    bool getObjectsInRoom_callback(social_segway::GetObjectsInRoom::Request &request, social_segway::GetObjectsInRoom::Response &response)
    {
        if (request.room.length() > 0)
        {
            auto objects = map->getObjectsInRoom(request.room.c_str());
            response.objects = objects;
            return true;
        }
        else
            return false;
    }

public:
    Semantic_node(ros::NodeHandle *nh)
    {
        nh->getParam("/semantic_node/compare", compare);

        if (compare)
        {
            changes_map = new Semantic_data_holder(nh, "map_changes.xml", true);
            map = new Semantic_data_holder(nh, "map.xml", false);
            changedDetectionSub = nh->subscribe("detected_objects", 1000, &Semantic_node::changeDetectionCallback, this);
            service_checkObject = nh->advertiseService("check_object", &Semantic_node::checkObject_callback, this);
            ROS_INFO_STREAM("Semantic Node mode: Comparator");

            auto allObjects = map->getAllObjects();
            //ROS_WARN_STREAM("size of object array " << sizeof(allObjects));
            for (auto object : allObjects)
            {
                timesFound.push_back(1);
                timesFound.push_back(1);
                detectedTimeStamp.push_back(ros::Time(0));
            }
            //ROS_INFO_STREAM("Times found size at beginning"<< timesFound.size());
            //ROS_INFO_STREAM("detected time stamp size"<< detectedTimeStamp.size());
        }
        else
        {
            map = new Semantic_data_holder(nh, "map.xml", true);
            detectionSub = nh->subscribe("detected_objects", 1000, &Semantic_node::detectionCallback, this);
            ROS_INFO_STREAM("Semantic Node mode: Mapping");
        }

        markerPub = nh->advertise<visualization_msgs::MarkerArray>("map_markers", 10);

        service_save_map = nh->advertiseService("save_map", &Semantic_node::saveMap_callback, this);
        service_reset_map = nh->advertiseService("reset_map", &Semantic_node::resetMap_callback, this);
        getRoomsSrv = nh->advertiseService("get_rooms", &Semantic_node::getRooms_callback, this);
        getLocationById = nh->advertiseService("get_location_by_id", &Semantic_node::getLocationById_callback, this);
        getObjectsInRoomSrv = nh->advertiseService("get_objects_in_room", &Semantic_node::getObjectsInRoom_callback, this);

        timer = nh->createTimer(ros::Duration(1.0), &Semantic_node::OneSecTimerCallback, this);
        idCounter = 0;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "semantic_node");
    ros::NodeHandle n;
    //testSemanticDataHolderClass(&n);
    //exit(10);
    Semantic_node *node;
    node = new Semantic_node(&n);
    ros::spin();

    return 0;
}