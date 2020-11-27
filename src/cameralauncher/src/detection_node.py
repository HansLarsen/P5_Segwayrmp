#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from darknet_ros_msgs.msg import BoundingBoxes
from cameralauncher.msg import Object, ObjectList
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

knownList = {24: "Item", 25: "Item", 26: "Item", 27: "Item", 28: "Item", 39: "Item", 40: "Item", 41: "Item", 42: "Item", 43: "Item", 44: "Item", 45: "Item", 46: "Item", 47: "Item", 48: "Item", 49: "Item", 50: "Item", 51: "Item", 52: "Item", 53: "Item", 54: "Item", 55: "Item", 56: "Furniture",
             57: "Furniture", 58: "Item", 59: "Furniture", 60: "Furniture", 61: "Furniture", 62: "Furniture", 63: "Item", 64: "Item", 65: "Item", 66: "Item", 67: "Item", 68: "Item", 69: "Furniture", 70: "Item", 71: "Furniture", 72: "Furniture", 73: "Item", 74: "Item", 75: "Item", 76: "Item", 78: "Item", 79: "Item"}

number_of_cameras = 2
camera_rotation = [cv2.ROTATE_90_CLOCKWISE, cv2.ROTATE_90_COUNTERCLOCKWISE]
camera_topics = ['/camera_l/color/image_raw', '/camera_r/color/image_raw']
camera_depth_topics = ['/camera_l/aligned_depth_to_color/image_raw',
                       '/camera_r/aligned_depth_to_color/image_raw']
camera_baselink_names = [
    'camera_l_color_optical_frame', 'camera_r_color_optical_frame']


def depth_pixel_to_metric(depth, pixel_x, pixel_y, intrinsics):
    #print intrinsics
    x = (((float(pixel_x)-intrinsics[2])/intrinsics[0])*float(depth))
    y = (((float(pixel_y)-intrinsics[5])/intrinsics[4])*float(depth))
    z = depth
    return Vector3(x, y, z)


if __name__ == '__main__':

    rospy.init_node('detection_node')

    #sub_camera_rgb = rospy.Subscriber('/camera/color/image_raw', sensor_msgs/Image, callback_rgb, queue_size=0)
    #sub_box = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, callback)

    pub_obj = rospy.Publisher('/detected_objects', ObjectList, queue_size=1)
    pub_to_yolo = rospy.Publisher('/camera/rgb/image_raw', Image, queue_size=1)
    current_camera_index = 0

    rospy.loginfo("Waiting for Camera Info")
    cam_info = rospy.wait_for_message(
        '/camera_l/aligned_depth_to_color/camera_info', CameraInfo)
    bridge = CvBridge()

    transform_broadcaster = tf2_ros.TransformBroadcaster()
    transform_message = geometry_msgs.msg.TransformStamped()

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    while not rospy.is_shutdown():
        rospy.loginfo("Waiting for Camera RGB {}".format(current_camera_index))

        rgb_image = rospy.wait_for_message(
            camera_topics[current_camera_index], Image)
        depth_image = rospy.wait_for_message(
            camera_depth_topics[current_camera_index], Image)

        #workingTime = rospy.Time(0)
        # try:
        #   trans = tfBuffer.lookup_transform(camera_baselink_names[current_camera_index], "camera_l_link", workingTime)
        # except:
        #   print "could not look up transform"
        #  continue

        rospy.loginfo("Done waiting")
        rospy.loginfo(current_camera_index)

        depthImageCv = CvBridge().imgmsg_to_cv2(depth_image, "passthrough")
        beforeYoloImg = CvBridge().imgmsg_to_cv2(rgb_image, "bgr8")
        afterRotationBeforeYolo = cv2.rotate(
            beforeYoloImg, camera_rotation[current_camera_index])
        beforeYolo2bridge = bridge.cv2_to_imgmsg(
            afterRotationBeforeYolo, "bgr8")

        beforeYolo2bridge.header = rgb_image.header
        pub_to_yolo.publish(beforeYolo2bridge)

        msg = {}

        recieveMessage = True
        restartFlag = False
        while(recieveMessage):
            try:
                msg = rospy.wait_for_message(
                    '/darknet_ros/bounding_boxes', BoundingBoxes, timeout=2)
            except:
                rospy.loginfo('Yolo did not find any objects')
                current_camera_index = current_camera_index + 1
                if (current_camera_index > number_of_cameras - 1):
                    current_camera_index = 0
                restartFlag = True
                break

            if (msg.image_header.stamp == beforeYolo2bridge.header.stamp):
                rospy.loginfo("Got a wierd timestamp")
                recieveMessage = False

        if (restartFlag):
            continue


        itemsInImageArray = ObjectList()

        #rospy.loginfo("I heard %s", len(msg.bounding_boxes))
        itemsInImageArray.header = msg.image_header
        for i in range(len(msg.bounding_boxes)):
            objecttypeTMP = knownList.get(msg.bounding_boxes[i].id)
            if objecttypeTMP == None:
                rospy.logerr("No rotation set")
                continue
            items = Object()
            items.type = objecttypeTMP
            items.objectClass = msg.bounding_boxes[i].Class
            items.id = 0

            centerBox_x = 0.0
            centerBox_y = 0.0
            centerBox_z = 0.0

            centerBox_x = (
                (msg.bounding_boxes[i].xmax-msg.bounding_boxes[i].xmin)/2) + msg.bounding_boxes[i].xmin
            centerBox_y = (
                (msg.bounding_boxes[i].ymax-msg.bounding_boxes[i].ymin)/2) + msg.bounding_boxes[i].ymin

            scalingFactor = (float(depthImageCv.shape[0])/float(beforeYoloImg.shape[0]), float(
                depthImageCv.shape[1])/float(beforeYoloImg.shape[1]))

            centerBox_x = int(centerBox_x*scalingFactor[1])
            centerBox_y = int(centerBox_y*scalingFactor[0])

            newImage = cv2.cvtColor(depthImageCv, cv2.COLOR_GRAY2BGR)

            if camera_rotation[current_camera_index] == cv2.ROTATE_90_CLOCKWISE:
                centerBox_x = depthImageCv.shape[0] - centerBox_x

            elif camera_rotation[current_camera_index] == cv2.ROTATE_90_COUNTERCLOCKWISE:
                centerBox_y = depthImageCv.shape[1] - centerBox_y
            else:
                rospy.logerr("Invalid rotation")
                continue

            # Den her er bagvendt fordi det er row,column altsaa y,x.
            centerBox_z = depthImageCv[centerBox_x, centerBox_y]

            if centerBox_z < 10.0:
                rospy.logerr("Short distance")
                continue

            #newImage = cv2.rectangle(newImage, (centerBox_y-10, centerBox_x-10), (centerBox_y+10, centerBox_x+10), (25000, 25000, 25000), 10)
            #newImage = cv2.resize(newImage, (newImage.shape[1] / 2, newImage.shape[0] / 2) )
            rospy.logwarn("got_this")
            #cv2.imshow("Cakesdatas", newImage)
            # cv2.waitKey(1)

            metric_point_to_cam = depth_pixel_to_metric(
                centerBox_z/1000.0, centerBox_y, centerBox_x, cam_info.K)
            items.transform.translation = metric_point_to_cam
            items.transform.rotation.w = 1

            transform_message.header = depth_image.header
            transform_message.child_frame_id = msg.bounding_boxes[i].Class
            transform_message.header.frame_id = camera_baselink_names[current_camera_index]
            transform_message.transform.translation = metric_point_to_cam
            transform_message.transform.rotation = copy.copy(
                items.transform.rotation)

            transform_broadcaster.sendTransform(transform_message)

            transform_rel_map = {}
            try:
                transform_rel_map = tfBuffer.lookup_transform(
                    "map", msg.bounding_boxes[i].Class, depth_image.header.stamp)

                itemsInImageArray.header.frame_id = transform_rel_map.header.frame_id
                items.transform = transform_rel_map.transform
                itemsInImageArray.objects.append(items)

            except:
                rospy.loginfo('error with looking up transform')
                continue

        itemsInImageArray.header.stamp = rospy.Time.now()

        current_camera_index = current_camera_index + 1

        if (current_camera_index > number_of_cameras - 1):
            current_camera_index = 0

        if (len(itemsInImageArray.objects) == 0):
            rospy.logerr("Array is empty")
            continue

        pub_obj.publish(itemsInImageArray)
