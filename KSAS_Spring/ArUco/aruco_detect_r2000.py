#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2, cv_bridge
import numpy as np
import cv2.aruco as aruco
import math
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import os
import timeit

class Aruco:
    
    def __init__(self):

      self.bridge = cv_bridge.CvBridge()
      self.image_sub = rospy.Subscriber('/camera1/color/image_raw', Image, self.image_callback)
      self.mtx = np.array([[640.0, 0, 320.0], [0, 640.0, 240.0], [0, 0, 1]])
      self.dist = np.array([[0.0, 0.0, 0.0, 0.0, 0.0]]) # k1, k2, p1, p2
      self.inv_mtx = np.linalg.inv(self.mtx)
      self.aruco_id = 0
      self.marker_size = 1.0  # [m]
      # self.pub = rospy.Publisher('object_pose', numpy_msg(Floats), queue_size=10)
      self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
      self.parameters = aruco.DetectorParameters_create()
      self.R_flip = np.zeros((3, 3), dtype=np.float32)
      self.R_flip[0, 0] = 1.0
      self.R_flip[1, 1] = -1.0
      self.R_flip[2, 2] = -1.0
      self.key = cv2.waitKey(1)
      self.rvec = np.array([[0], [0], [0]])
      self.tvec = np.array([[0], [0], [0]])
      self.Axis =  np.array([[0.0, 0.0, 0.0], [0.4, 0.0, 0.0], [0.0, 0.4, 0.0], [0.0, 0.0, 0.4]])
      self.fps = 30.0
      self.cur = 0.0
      self.pre = 0.0


    def image_callback(self, msg):

      image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

      image = self.aruco_detection(image)

      cv2.imshow("R2000 Image", image)

      self.Key_Control()

      return image
    
    def End():

        rospy.loginfo("End of Process")    

    def Key_Control(self):
       
       self.key = cv2.waitKey(1)

       if self.key == 27:
          
          cv2.destroyAllWindows()
          print("---end---")
          os._exit(1)


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

    
    def drawAxis(self, img):

        axispts, jacobian = cv2.projectPoints(self.Axis, self.rvec, self.tvec, self.mtx, self.distort_mtx)

        axispts = axispts.astype(int)

        img = cv2.line(img, tuple(axispts[0].ravel()), tuple(axispts[1].ravel()), (255, 0, 0), 3)
        img = cv2.line(img, tuple(axispts[0].ravel()), tuple(axispts[2].ravel()), (0, 255, 0), 3)
        img = cv2.line(img, tuple(axispts[0].ravel()), tuple(axispts[3].ravel()), (0, 0, 255), 3)

        return img


    def aruco_detection(self, image):

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # blur = cv2.GaussianBlur(gray, (5, 5), 0)

        # ret, gray = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=self.aruco_dict, parameters=self.parameters)

        self.cur = timeit.default_timer()

        self.fps = 1.0 / max((self.cur - self.pre), 0.000001)

        self.pre = self.cur

        cv2.putText(image, str(self.fps), (400, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

        if np.all(ids != None) and ids[0] == self.aruco_id:
          
            try:
                
                ret = aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.mtx, self.dist)

                self.rvec, self.tvec = ret[0][0, 0, :], ret[1][0, 0, :]

                aruco.drawDetectedMarkers(image, corners)

                aruco.drawAxis(image, self.mtx, self.dist, self.rvec, self.tvec, 0.1)

                str_Position_Center = "Position X=%0.4f Y=%0.4f Z=%0.4f" % (self.tvec[0], self.tvec[1], self.tvec[2])

                cv2.putText(image, str_Position_Center, (0, 30), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

                R_ct = np.matrix(cv2.Rodrigues(self.rvec)[0])

                R_tc = R_ct.T
                
                Pitch, Yaw, Roll = self.rotationMat2EulerAngle(R_tc * self.R_flip)

                str_Attitude = "Attitude R=%0.4f P=%0.4f Y=%0.4f" % (math.degrees(Roll), math.degrees(Pitch), math.degrees(Yaw))

                cv2.putText(image, str_Attitude, (0, 60), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

                # rospy.loginfo('publishing object frame')
                Pose = np.array([self.tvec[0], self.tvec[1], self.tvec[2], math.degrees(Roll), math.degrees(Pitch), math.degrees(Yaw)], dtype=np.float32)

            except Exception as exception:
                
                rospy.loginfo(exception)

        return image


if __name__ == '__main__':
    
    try:
      
        rospy.init_node('R2000_relative_position', anonymous=True)
        R2000_Aruco_Detector = Aruco()
        rospy.spin()

    except rospy.ROSInterruptException as Exception:
        
        print(Exception)
        pass