#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from darknet_ros_msgs.msg import BoundingBoxes
from cameralauncher.msg import objects, objectstruct
from sensor_msgs.msg import Image, CameraInfo
import numpy
import cv2
from cv_bridge import CvBridge
import tf2_py
from geometry_msgs.msg import Vector3
import struct
import copy
import tf2_ros
import geometry_msgs.msg
import tf2_geometry_msgs

#import pyrealsense2

knownList = {24: "item", 25: "item", 26: "item", 27: "item", 28: "item", 39: "item", 40: "item", 41: "item", 42: "item", 43: "item", 44: "item", 45: "item", 46: "item", 47: "item", 48: "item", 49: "item", 50: "item", 51: "item", 52: "item", 53: "item", 54: "item", 55: "item", 56: "furniture",
             57: "furniture", 58: "item", 59: "furniture", 60: "furniture", 61: "furniture", 62: "furniture", 63: "item", 64: "item", 65: "item", 66: "item", 67: "item", 68: "item", 69: "furniture", 70: "item", 71: "furniture", 72: "furniture", 73: "item", 74: "item", 75: "item", 76: "item", 78: "item", 79: "item"}

number_of_cameras = 2
camera_rotation = [cv2.ROTATE_90_COUNTERCLOCKWISE, cv2.ROTATE_90_COUNTERCLOCKWISE]
camera_topics = ['/camera/color/image_raw', '/camera/color/image_raw']
camera_depth_topics = ['/camera/aligned_depth_to_color/image_raw', '/camera/aligned_depth_to_color/image_raw']
camera_baselink_names = ['camera_color_optical_frame', 'camera_color_optical_frame']

def depth_pixel_to_metric(depth, pixel_x, pixel_y, intrinsics):
    #print intrinsics
    x = (((float(pixel_x)-intrinsics[2])/intrinsics[0])*float(depth))
    y = (((float(pixel_y)-intrinsics[5])/intrinsics[4])*float(depth))
    z = depth
    return Vector3(x,y,z)


def yolo_callback(msg, depth_img, camera_info):
    itemsInImageArray = objects()

    #rospy.loginfo("I heard %s", len(msg.bounding_boxes))
    itemsInImageArray.header = msg.image_header

    for i in range(len(msg.bounding_boxes)):
        objecttypeTMP = knownList.get(msg.bounding_boxes[i].id)
        if objecttypeTMP == None:
            continue
        items = objectstruct()
        items.type = objecttypeTMP
        items.Class = msg.bounding_boxes[i].Class
        items.id = 0
        items.location_relative_to_map.translation.x = 0
        items.location_relative_to_map.translation.y = 0
        items.location_relative_to_map.translation.z = 0
        items.location_relative_to_map.rotation.w = 1
        itemsInImageArray.objectsarray.append(items)

    #print "____________"
    #print itemsInImageArray
    pub_obj.publish(itemsInImageArray)


if __name__ == '__main__':

    rospy.init_node('detection_node')

    #sub_camera_rgb = rospy.Subscriber('/camera/color/image_raw', sensor_msgs/Image, callback_rgb, queue_size=0)
    #sub_box = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, callback)

    pub_obj = rospy.Publisher('/Detected_Objects', objects, queue_size=1)
    pub_to_yolo = rospy.Publisher('/camera/rgb/image_raw', Image, queue_size=1)


    rospy.loginfo("Waiting for Camera Info")
    cam_info = rospy.wait_for_message(
        '/camera/aligned_depth_to_color/camera_info', CameraInfo)
    bridge = CvBridge()

    transform_broadcaster = tf2_ros.TransformBroadcaster()
    transform_message = geometry_msgs.msg.TransformStamped()

    current_camera_index = 0

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    while not rospy.is_shutdown():
        #rospy.loginfo("Waiting for Camera RGB {}".format(current_camera_index))

        rgb_image = rospy.wait_for_message(camera_topics[current_camera_index], Image)
        depth_image = rospy.wait_for_message(camera_depth_topics[current_camera_index], Image)
        try:
            trans = tfBuffer.lookup_transform(camera_baselink_names[current_camera_index], "camera_link", rospy.Time(0))
        except:
            print "could not look up transform"
            continue
        
        rospy.loginfo("Done waiting")

        depthImageCv = CvBridge().imgmsg_to_cv2(depth_image, "passthrough")
        beforeYoloImg = CvBridge().imgmsg_to_cv2(rgb_image, "bgr8")
        afterRotationBeforeYolo = cv2.rotate(beforeYoloImg, camera_rotation[current_camera_index])
        beforeYolo2bridge = bridge.cv2_to_imgmsg(afterRotationBeforeYolo,"bgr8")

        pub_to_yolo.publish(beforeYolo2bridge)

        msg = {}
        try:
            msg = rospy.wait_for_message('/darknet_ros/bounding_boxes', BoundingBoxes, timeout=2)
        except:
            rospy.loginfo('Yolo did not find any objects')
            continue
        
        itemsInImageArray = objects()

        #rospy.loginfo("I heard %s", len(msg.bounding_boxes))
        itemsInImageArray.header = msg.image_header
        for i in range(len(msg.bounding_boxes)):
            objecttypeTMP = knownList.get(msg.bounding_boxes[i].id)
            if objecttypeTMP == None:
                continue
            items = objectstruct()
            items.type = objecttypeTMP
            items.Class = msg.bounding_boxes[i].Class
            items.id = 0

            centerBox_x = 0.0
            centerBox_y = 0.0
            centerBox_z = 0.0

            centerBox_x = (msg.bounding_boxes[i].xmax-msg.bounding_boxes[i].xmin)/2 +msg.bounding_boxes[i].xmin
            centerBox_y = (msg.bounding_boxes[i].ymax-msg.bounding_boxes[i].ymin)/2 +msg.bounding_boxes[i].ymin

            if (centerBox_y > beforeYoloImg.shape[0] or centerBox_x > beforeYoloImg.shape[1]):
                continue

            if camera_rotation[current_camera_index] == cv2.ROTATE_90_CLOCKWISE:
                centerBox_x = beforeYoloImg.shape[0] - centerBox_x
            elif camera_rotation[current_camera_index] == cv2.ROTATE_90_COUNTERCLOCKWISE:
                centerBox_y = beforeYoloImg.shape[1] - centerBox_y
            else:
                rospy.logerr("Invalid rotation")
                continue

            centerBox_z = depthImageCv[centerBox_y, centerBox_x]

            metric_point_to_cam = depth_pixel_to_metric(centerBox_z/1000.0,centerBox_y,centerBox_x, cam_info.K)
            #print metric_point_to_cam
            items.location_relative_to_map.translation = metric_point_to_cam
            items.location_relative_to_map.rotation.w = 1
            itemsInImageArray.objectsarray.append(items)

            transform_message.header = msg.image_header
            transform_message.child_frame_id = msg.bounding_boxes[i].Class
            transform_message.header.frame_id = camera_baselink_names[current_camera_index]
            transform_message.transform.translation = metric_point_to_cam
            transform_message.transform.rotation = copy.copy(items.location_relative_to_map.rotation)

            transform_broadcaster.sendTransform(transform_message)

            transform_rel_map ={}
            try:
                transform_rel_map = tfBuffer.lookup_transform("camera_link",msg.bounding_boxes[i].Class,rospy.Time(0))
            except:
                rospy.loginfo('error with looking up transform')
                continue
            
            itemsInImageArray.header.frame_id = transform_rel_map.header.frame_id
            items.location_relative_to_map = transform_rel_map.transform

        itemsInImageArray.header.stamp = rospy.Time.now()
        pub_obj.publish(itemsInImageArray)

        current_camera_index = current_camera_index + 1

        if (current_camera_index > number_of_cameras - 1):
            current_camera_index = 0