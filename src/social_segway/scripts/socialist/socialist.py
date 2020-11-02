1
import rospy
import Talker

from std_msgs.msg import String

def speak_text_callback(data):
    pass

def socialist():
    rospy.init_node("socialist")
    rospy.Subscriber("speak_text", String, speak_text_callback)


    rospy.spin()



if __name__ == '__main__':
    socialist()
    