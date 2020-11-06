#include <ros/ros.h>
#include <ros/package.h>
#include "xml_lib/xml_lib.h"
#include <tinyxml2.h> //only here for intellisense, already included in xml_lib/xml_lib.h - struggles were had!
#include "social_segway/Object.h"
#include "geometry_msgs/Transform.h"
#include <vector>
#include <string>

#ifndef XMLCheckResult
#define XMLCheckResult(a_eResult)         \
    if (a_eResult != XML_SUCCESS)         \
    {                                     \
        printf("Error: %i\n", a_eResult); \
    }
#endif

using namespace tinyxml2;

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
    map->addRoom("dining room");
    map->addRoom("bedroom");
    map->addRoom("living room");

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

    map->addObjectToRoom("dining room", object);
    object.id = 2;
    map->addObjectToRoom("dining room", object);

    auto objects = map->getObjectsInRoom("dining room");
    std::cout << objects.size() << std::endl;
    for (auto object : objects)
    {
        std::cout << object << ", ";
    }
    std::cout << std::endl;

    map->saveMap();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "semantic_node");
    return 0;
}
