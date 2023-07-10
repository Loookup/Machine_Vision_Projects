#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes
import cv2, cv_bridge
import numpy as np
import math
import cv2.aruco as aruco
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import Int8
import os
import timeit
import copy


class Docking:

    
    def __init__(self):

        self.aruco_id = 0
        self.marker_size = 0.25  # [m]
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
        self.parameters = aruco.DetectorParameters_create()

        self.system_mat = Float64MultiArray()
        self.Dimension = MultiArrayDimension()
        self.Dimension.size = 8
        self.Dimension.stride = 1
        self.Dimension.label = "Info"
        self.system_mat.layout.dim.append(self.Dimension)

        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/camera1/color/image_raw', Image, self.image_callback)
        self.depth_sub = rospy.Subscriber('/camera1/depth/image_raw', Image, self.depth_callback)

        self.YOLO_sub = sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes , self.YOLO_callback)
        self.center_x = 320
        self.center_y = 240

        self.pub = rospy.Publisher('System_Controller', Float64MultiArray, queue_size=10)
    
        self.mtx = np.array([[640.0, 0, 320.0], [0, 640.0, 240.0], [0, 0, 1]])
        self.dist = np.array([[0.0, 0.0, 0.0, 0.0, 0.0]]) # k1, k2, p1, p2
        self.R_flip = np.zeros((3, 3), dtype=np.float32)
        self.R_flip[0, 0] = 1.0
        self.R_flip[1, 1] = -1.0
        self.R_flip[2, 2] = -1.0
        self.key = cv2.waitKey(1)
        self.fps = 30.0
        self.cur = 0.0
        self.pre = 0.0
        self.objs = np.array([[-0.5, 0.5, 0.0], [0.5, 0.5, 0.0], [0.5, -0.5, 0.0], [-0.5, -0.5, 0.0]])
        self.rvec = np.array([[0], [0], [0]])
        self.tvec = np.array([[0], [0], [0]])
        self.Roll = 0
        self.Pitch = 0
        self.Yaw = 0
        self.Mode = 0
        self.distance = None


    def YOLO_callback(self, msg):

        self.Mode = 1

        for box in msg.bounding_boxes:

            self.center_x = (box.xmin + box.xmax)/2
            self.center_y = (box.ymin + box.ymax)/2



    def depth_callback(self, msg):

        try:

            depth_raw_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

        except cv_bridge.CvBridgeError as e:

            print(e)

        self.distance = depth_raw_image[320][240]

        # depth_raw_image = cv2.normalize(depth_raw_image, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)

        # self.depth_pub.publish(distance)

        # cv2.imshow("Depth Image", depth_raw_image)

        # self.Key_Control()



    def image_callback(self, msg):

        self.Initiator()

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        image = self.aruco(image)

        image = self.Move(image)

        self.pub.publish(self.system_mat)

        cv2.imshow("Image", image)

        self.Key_Control()


    def Move(self, image):

            

        if self.Mode == 0:

            self.system_mat.data = [self.Mode, 0, 0, 30, 0, 0, 0, 2]

            str_YOLO = "YOLO None"

            cv2.putText(image, str_YOLO, (10, 150), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

        elif self.Mode == 1:

            # self.system_mat.data = [self.Mode, self.center_x-320, self.center_y-240, 30, 0, 0, 0, 0]

            self.system_mat.data = [self.Mode, self.center_x-320, self.center_y-240, 30, 0, 0, 0, 2]

            str_YOLO = "YOLO X=%0.4f Y=%0.4f" % (math.floor(self.center_x), math.floor(self.center_y))

            cv2.putText(image, str_YOLO, (10, 150), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

            # print(str(320-self.center_x) + " & "+ str(240-self.center_y))

            # self.pub.publish(self.system_mat)

        elif self.Mode == 2:

            self.system_mat.data = [self.Mode, self.tvec[0], self.tvec[1], self.tvec[2], self.Roll, self.Pitch, self.Yaw, self.distance]

            str_YOLO = "YOLO X=%0.4f Y=%0.4f" % (math.floor(self.center_x), math.floor(self.center_y))

            cv2.putText(image, str_YOLO, (10, 150), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

            # self.pub.publish(self.system_mat)

        else:

            pass

        if self.distance <= 0.7:

            self.system_mat.data = [self.Mode, 0, 0, 0, 0, 0, 0, self.distance]

        return image


    def Initiator(self):

        self.Mode = 0
        self.rvec = np.array([[0], [0], [0]])
        self.tvec = np.array([[0], [0], [0]])
        self.Roll = 0
        self.Pitch = 0
        self.Yaw = 0


    def Key_Control(self):
       
        self.key = cv2.waitKey(1)

        if self.key == 27:

            cv2.destroyAllWindows()

            rospy.loginfo("---end---")

            os._exit(1)

        elif self.key == ord('i') or self.key == ord('I'):
           
            rospy.loginfo("Initiate Robot Control")

            self.Mode = 1

        elif self.key == ord('f') or self.key == ord('F'):
   
            rospy.loginfo("Fixed Number Sequence")

            self.Mode = 2

        elif self.key == ord('q') or self.key == ord('Q'):

            rospy.loginfo("Back to Initial Position")

            self.Mode = 0


  # Verifying R
    def isRotationMatrix(self, R):

        Rt = np.transpose(R)

        Identity = np.dot(Rt, R)

        I = np.identity(3, dtype=R.dtype)

        n = np.linalg.norm(I - Identity)

        return n < 1e-6  # Error


    # Rotation Matrix to Euler Angles (Opencv Algorithm)
    def rotationMat2EulerAngle(self, R):

        assert (self.isRotationMatrix(R))

        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

        singular = sy < 1e-6

        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0

        return np.array([x, y, z])
    
    

    def aruco(self, origin):

        image = copy.deepcopy(origin)

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=self.aruco_dict, parameters=self.parameters, cameraMatrix=self.mtx, distCoeff=self.dist)

        self.cur = timeit.default_timer()

        self.fps = 1.0 / max((self.cur - self.pre), 0.000001)

        self.pre = self.cur

        cv2.putText(image, str(self.fps), (400, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

        if np.all(ids != None) and ids[0] == self.aruco_id:

            self.Mode = 2

            try:

                ret = aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.mtx, self.dist)

                self.rvec, self.tvec, Obj_points_2D = ret[0][0, 0, :], ret[1][0, 0, :], ret[2]

                aruco.drawDetectedMarkers(image, corners)

                aruco.drawAxis(image, self.mtx, self.dist, self.rvec, self.tvec, 0.1)

                str_Position = "Position X=%0.4f Y=%0.4f Z=%0.4f" % (self.tvec[0], self.tvec[1], self.tvec[2])

                cv2.putText(image, str_Position, (10, 60), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

                R_ct = np.matrix(cv2.Rodrigues(self.rvec)[0])
                R_tc = R_ct.T

                Pitch, Yaw, Roll = self.rotationMat2EulerAngle(R_tc * self.R_flip)

                str_Attitude = "Attitude R=%0.4f P=%0.4f Y=%0.4f" % (math.degrees(Roll), math.degrees(Pitch), math.degrees(Yaw))

                cv2.putText(image, str_Attitude, (10, 90), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

                cv2.putText(image, "Depth : "+str(self.distance), (10, 120), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

                self.Roll, self.Pitch, self.Yaw = Roll, Pitch, Yaw

            except rospy.ROSInterruptException as Exception:

                print(Exception)
                pass

        cv2.putText(image, str(self.Mode), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

        return image


if __name__ == '__main__':
    
    try:
      
        rospy.init_node('Sat Camera', anonymous=True)

        docking = Docking()

        rospy.spin()

    except rospy.ROSInterruptException as Exception:
        
        print(Exception)
        pass