#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import Header
from std_msgs.msg import String

def callback(data):
    xim = 1
    for box in data.bounding_boxes:
        xim = box.xmin
        # rospy.loginfo("Xmin: {}, Xmax: {} Ymin: {}, Ymax: {}".format(box.xmin, box.xmax, box.ymin, box.ymax))
    print(xim)

rospy.init_node('listener')
sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes , callback)
rospy.spin()

# def main():
#     while not rospy.is_shutdown():
#         rospy.init_node('listener', anonymous = True)
#         sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes , callback)
#         rospy.spin()

# if __name__ == '__main__':
#     try :
#         main()
#     except rospy.ROSInterruptException:
#         pass