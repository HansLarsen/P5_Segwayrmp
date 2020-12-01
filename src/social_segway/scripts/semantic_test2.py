#!/usr/bin/env python



import rospy

from cameralauncher.msg import Object
from cameralauncher.msg import ObjectList
from std_srvs.srv import *
from time import sleep
from copy import copy
    


def main():
    rospy.init_node('semantic_node_tester', anonymous=True)
    pub = rospy.Publisher('/detected_objects', ObjectList, queue_size=10)
    sleep(1)
    msg = ObjectList()
    
    msgObj = Object()
    msgObj.type = "Furniture"
    msgObj.objectClass = "Chair"
    msgObj.transform.translation.x = 0
    msgObj.transform.translation.y = 4
    msgObj.transform.translation.z = 0
    msg.objects.append(msgObj)

    msgObj = Object()
    msgObj.type = "Item"
    msgObj.objectClass = "Bottle"
    msgObj.transform.translation.x = 5
    msgObj.transform.translation.y = 5
    msgObj.transform.translation.z = 0
    msg.objects.append(msgObj)


    print("publishing")
    pub.publish(msg)
    print("published")

    rospy.wait_for_service("/save_map")
    sleep(2)
    try:
        save_map = rospy.ServiceProxy("/save_map", Trigger)
        response = save_map()
        print(response)
    except rospy.ServiceException as e:
        print("failed to call /save_map: %s" %e)

if __name__ == "__main__":
    main()