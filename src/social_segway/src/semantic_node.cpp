#include <ros/ros.h>
#include <ros/package.h>
#include "xml_lib/xml_lib.h"
#include "social_segway/Object.h"
#include "social_segway/ObjectList.h"
#include "geometry_msgs/Transform.h"
#include <vector>
#include <string>

using namespace xml_lib;

#ifndef XMLCheckResult
#define XMLCheckResult(a_eResult)         \
    if (a_eResult != XML_SUCCESS)         \
    {                                     \
        printf("Error: %i\n", a_eResult); \
    }
#endif

// https://shilohjames.wordpress.com/2014/04/27/tinyxml2-tutorial/
class semantic_data_holder
{
    const char *file_name;
    XMLDocument *doc;
    XMLNode *root;

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

    const char *findRoomByLocation()
    {
    }

public:
    semantic_data_holder()
    {
        std::string pkg_path = ros::package::getPath("social_segway");
        pkg_path += "/xml/map.xml";
        std::string file_path = "~" + pkg_path;
        file_name = file_path.c_str();
        file_name = "map.xml";

        //ROS_INFO_STREAM("Creating map in: ");
        //ROS_INFO_STREAM(file_name);
        doc = new XMLDocument();

        XMLError error = doc->LoadFile(file_name);
        if (error == XML_SUCCESS)
        {
            //delete old map
            doc->Clear();
        }
        root = doc->NewElement("map");
        doc->InsertFirstChild(root);
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

        return true;
    }

    void saveMap()
    {
        ROS_INFO_STREAM("Saving semantic map");
        XMLError error = doc->SaveFile(file_name);
        XMLCheckResult(error);
    }
};

void testSemanticDataHolderClass()
{
    semantic_data_holder *map;
    map = new semantic_data_holder();
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


class Semantic_node{

    float allowedDeviation = 5; // idk just some value for now
    float allowedDeviation2;

    ros::Subscriber detectionSub;
    ros::Timer timer;
    semantic_data_holder* map;

    void detectionCallback(const social_segway::ObjectList& data){

        for (auto detectedObject : data.objects){
            auto allObjects = map->getAllObjects();
            for (auto object : allObjects){ 

                if (((detectedObject.transform.translation.x < object.transform.translation.x + allowedDeviation &&
                    detectedObject.transform.translation.x > object.transform.translation.x - allowedDeviation) ||
                    (detectedObject.transform.translation.y < object.transform.translation.y + allowedDeviation &&
                    detectedObject.transform.translation.y > object.transform.translation.y - allowedDeviation) ||
                    (detectedObject.transform.translation.z < object.transform.translation.z + allowedDeviation &&
                    detectedObject.transform.translation.z > object.transform.translation.z - allowedDeviation)) && 
                    (detectedObject.objectClass == object.objectClass &&
                    detectedObject.type == object.type)){
                    
                    // Merge items: detectedObject and object (or ignore detectedObject?)
                    continue;
                    mergeObjects(detectedObject, object);
                }
                else{ // add item to list
                    social_segway::Object newObject;
                    newObject.id = detectedObject.id;
                    newObject.transform.translation.x = detectedObject.transform.translation.x;
                    newObject.transform.translation.y = detectedObject.transform.translation.y;
                    newObject.transform.translation.z = detectedObject.transform.translation.z;
                    newObject.type = detectedObject.type;
                    newObject.objectClass = detectedObject.objectClass;

                    map->addObjectByPosition(newObject);
                }
            }        
        }    
    }

    void checkOnTopTimerCallback(const ros::TimerEvent&){ // Check if LooseObject is on top of Furniture object

        auto allFurniture = map->getAllFurniture();
        auto allLooseObjects = map->getLooseObjects();

        for (auto Furniture : allFurniture){
            for (auto LooseObject: allLooseObjects){

                if (LooseObject.transform.translation.z > Furniture.transform.translation.z){

                    if (Furniture.objectClass == "dinner_table")
                        allowedDeviation2 = 1; // estimated radius of the Furniture upper surface
                    else if(Furniture.objectClass == "shelve")
                        allowedDeviation2 = 2;
                    else if(Furniture.objectClass == "coffee_table")
                        allowedDeviation2 = 0.5;                    

                    if ((LooseObject.transform.translation.x < Furniture.transform.translation.x + allowedDeviation2 &&
                        LooseObject.transform.translation.x > Furniture.transform.translation.x - allowedDeviation2) ||
                        (LooseObject.transform.translation.y < Furniture.transform.translation.y + allowedDeviation2 &&
                        LooseObject.transform.translation.y > Furniture.transform.translation.y - allowedDeviation2)){

                        map->placeObjectOnFurniture(LooseObject, Furniture);
                    }
                }
            }
        }
    }

    bool mergeObjects(social_segway::Object detectedObject, social_segway::Object object){
        //Merge somehow
    }

public:

    Semantic_node(ros::NodeHandle* nh)
    {
        map = new semantic_data_holder();
        detectionSub = nh->subscribe("Detected_Objects", 1000, &Semantic_node::detectionCallback, this);
        timer = nh->createTimer(ros::Duration(1.0), &Semantic_node::checkOnTopTimerCallback, this);
    }


};




int main(int argc, char **argv)
{
    ros::init(argc, argv, "semantic_node");
    ros::NodeHandle n;
    ros::spin();

    return 0;
}