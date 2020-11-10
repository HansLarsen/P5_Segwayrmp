#include <ros/ros.h>
#include <ros/package.h>
#include "xml_lib/xml_lib.h"
#include "social_segway/Object.h"
#include "social_segway/ObjectList.h"
#include "geometry_msgs/Transform.h"
#include "std_srvs/Trigger.h"
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

    const char *file_name;
    XMLDocument *doc;
    XMLNode *root;
    XMLDocument *roomDoc;
    std::vector<RoomStruct> roomVector;
    ros::ServiceServer service_save_map;
    ros::ServiceServer service_reset_map;

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

    const char *getRoomByPosition(float x, float y)
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
                return room.name.c_str();
        }
        return "Unknown";
    }

    void saveMap(bool); //placeholder function
    void resetMap()
    {
        root = doc->NewElement("map");
        doc->InsertFirstChild(root);
        addRoom("Unknown");
    }

    bool saveMap_callback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
    {
        saveMap();
        response.success = true;
        response.message = "Saved map";
        return true;
    }
    bool resetMap_callback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
    {
        resetMap();
        response.success = true;
        response.message = "Reset map";
        return true;
    }

public:
    Semantic_data_holder(ros::NodeHandle *nh)
    {
        std::string pkg_path = ros::package::getPath("social_segway");
        pkg_path += "/xml/map.xml";
        std::string file_path = "~" + pkg_path;
        file_name = file_path.c_str();
        file_name = "map.xml";

        service_save_map = nh->advertiseService("save_map", &Semantic_data_holder::saveMap_callback, this);
        service_reset_map = nh->advertiseService("reset_map", &Semantic_data_holder::resetMap_callback, this);
        //ROS_INFO_STREAM("Creating map in: ");
        //ROS_INFO_STREAM(file_name);
        doc = new XMLDocument();

        XMLError error = doc->LoadFile(file_name);
        if (error == XML_SUCCESS)
        {
            //delete old map
            doc->Clear();
        }
        resetMap();

        //Read room document:
        auto room_map_name = "rooms.xml";
        roomDoc = new XMLDocument();
        error = roomDoc->LoadFile(room_map_name);
        if (error != XML_SUCCESS)
        {
            ROS_ERROR_STREAM("FAILED TO OPEN \"rooms.xml\" KILLING SEMANTIC NODE");
            exit(-1);
        }
        auto room = roomDoc->FirstChildElement();
        while (room != NULL)
        {
            RoomStruct roomStr;
            roomStr.name = room->Name();

            auto posElement = room->FirstChildElement();
            for (int i = 0; i < 2; i++)
            {
                Point point;
                point.x = posElement->FloatAttribute("x");
                point.y = posElement->FloatAttribute("y");
                roomStr.points[i] = point;
            }
            room = room->NextSiblingElement();
            roomVector.push_back(roomStr);
        }
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

        std::cout << "insert: " << furnitureEle->InsertEndChild(itemEle) << "\n\n";
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
                        objects.push_back(XMLElementToObject(items));
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

    bool addObjectByPosition(social_segway::Object object)
    {
        auto room = getRoomByPosition(object.transform.translation.x, object.transform.translation.y);
        return addObjectToRoom(room, object);
    }
    void saveMap()
    {
        ROS_INFO_STREAM("Saving semantic map");
        XMLError error = doc->SaveFile(file_name);
        XMLCheckResult(error);
    }
};

void testSemanticDataHolderClass(ros::NodeHandle *nh)
{
    Semantic_data_holder *map;
    map = new Semantic_data_holder(nh);
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

    float allowedDeviation = 5; // idk just some value for now
    float allowedDeviation2;
    int idCounter;
    double distance;
    float x1, y1, z1, x2, y2, z2;

    ros::Subscriber detectionSub;
    ros::Timer timer;
    Semantic_data_holder *map;

    void detectionCallback(const social_segway::ObjectList &data)
    {
        auto allObjects = map->getAllObjects();
        if (allObjects.size() == 0) // no objects yet, add first:
        {
            // add item to list
            detectedObject.id = idCounter;
            idCounter++;
            map->addObjectToRoom("Unknown", detectedObject);
            continue;
        }
        for (auto detectedObject : data.objects)
        {
            x1 = detectedObject.transform.translation.x;
            y1 = detectedObject.transform.translation.y;
            z1 = detectedObject.transform.translation.z;

            auto allObjects = map->getAllObjects();
            for (auto object : allObjects)
            {
                x2 = object.transform.translation.x;
                y2 = object.transform.translation.y;
                z2 = object.transform.translation.z;

                distance = std::hypot(std::hypot(x1 - x2, y1 - y2), z1 - z2);

                if (distance < allowedDeviation && detectedObject.objectClass == object.objectClass && detectedObject.type == object.type)
                {
                    // Merge items: detectedObject and object (or ignore detectedObject?)
                    continue;
                    mergeObjects(detectedObject, object);
                }
                else
                { // add item to list
                    detectedObject.id = idCounter;
                    idCounter++;
                    map->addObjectToRoom("Unknown", detectedObject);
                }
            }
        }
    }

    void checkOnTopTimerCallback(const ros::TimerEvent &)
    { // Check if LooseObject is on top of Furniture object

        auto allFurniture = map->getAllFurniture();
        for (auto Furniture : allFurniture)
        {
            x1 = Furniture.transform.translation.x;
            y1 = Furniture.transform.translation.y;
            z1 = Furniture.transform.translation.z;

            auto allLooseObjects = map->getLooseObjects();
            for (auto LooseObject : allLooseObjects)
            {
                x2 = LooseObject.transform.translation.x;
                y2 = LooseObject.transform.translation.y;
                z2 = LooseObject.transform.translation.z;

                distance = std::hypot(x1 - x2, y1 - y2);

                if (z2 > z1)
                { // if higher

                    if (Furniture.objectClass == "dinner_table")
                        allowedDeviation2 = 1; // estimated radius of the Furniture upper surface
                    else if (Furniture.objectClass == "shelve")
                        allowedDeviation2 = 2;
                    else if (Furniture.objectClass == "coffee_table")
                        allowedDeviation2 = 0.5;
                    //ect

                    if (distance < allowedDeviation2)
                    {

                        map->placeObjectOnFurniture(LooseObject, Furniture);
                    }
                }
            }
        }
    }

    bool mergeObjects(social_segway::Object detectedObject, social_segway::Object object)
    {
        //Merge somehow
    }

public:
    Semantic_node(ros::NodeHandle *nh)
    {
        map = new Semantic_data_holder(nh);
        detectionSub = nh->subscribe("detected_objects", 1000, &Semantic_node::detectionCallback, this);
        timer = nh->createTimer(ros::Duration(1.0), &Semantic_node::checkOnTopTimerCallback, this);
        idCounter = 1;
    }

    ~Semantic_node()
    {
        map->saveMap();
    }
};

static volatile int keepRunning = 1;

void intHandler(int dummy)
{
    keepRunning = 0;
}

int main(int argc, char **argv)
{
    signal(SIGINT, intHandler);
    ros::init(argc, argv, "semantic_node");
    ros::NodeHandle n;
    Semantic_node *node;
    node = new Semantic_node(&n);

    while (keepRunning)
    {
        ros::spin();
    }

    return 0;
}