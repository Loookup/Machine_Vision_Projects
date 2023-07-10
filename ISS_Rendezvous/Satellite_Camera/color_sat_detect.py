#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import cv2, cv_bridge
import numpy as np
import math
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import os
import timeit
import copy


class Sat_Detector:

    
    def __init__(self):

        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/camera1/color/image_raw', Image, self.image_callback)
        # self.depth_sub = rospy.Subscriber('/camera1/depth/image_raw', Image, self.depth_callback)
        # self.error_pub = rospy.Publisher('/sat_pose', numpy_msg(Floats), queue_size=10)
        self.mtx = np.array([[640.0, 0, 320.0], [0, 640.0, 240.0], [0, 0, 1]])
        self.dist = np.array([[0.0, 0.0, 0.0, 0.0, 0.0]]) # k1, k2, p1, p2
        self.R_flip = np.zeros((3, 3), dtype=np.float32)
        self.R_flip[0, 0] = 1.0
        self.R_flip[1, 1] = -1.0
        self.R_flip[2, 2] = -1.0
        self.key = cv2.waitKey(1)
        self.Axis =  np.array([[0.0, 0.0, 0.0], [0.4, 0.0, 0.0], [0.0, 0.4, 0.0], [0.0, 0.0, 0.4]])
        self.fps = 30.0
        self.cur = 0.0
        self.pre = 0.0
        self.objs = np.array([[-0.5, 0.5, 0.0], [0.5, 0.5, 0.0], [0.5, -0.5, 0.0], [-0.5, -0.5, 0.0]])
        self.rvec = np.array([[0], [0], [0]])
        self.tvec = np.array([[0], [0], [0]])
        self.Roll = 0
        self.Pitch = 0
        self.Yaw = 0
        self.Axis =  np.array([[0.0, 0.0, 0.0], [0.4, 0.0, 0.0], [0.0, 0.4, 0.0], [0.0, 0.0, 0.4]])
        self.fps = 30.0
        self.cur = 0.0
        self.pre = 0.0
        self.Relative_Coord = np.array([0, 0, 0, 0, 0, 0], dtype=np.float32)
        self.Mode = 0
        self.Pre_Box_Points = np.array([(0, 0), (0, 0), (0, 0), (0, 0)], dtype=np.float64)


    
    def depth_callback(self, msg):

        try:

            depth_raw_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

        except cv_bridge.CvBridgeError as e:

            print(e)

        depth_raw_image = cv2.normalize(depth_raw_image, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)

        cv2.imshow("Depth Image", depth_raw_image)

        self.Key_Control()


    def image_callback(self, msg):

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        image, th = self.sat_detection(image)

        cv2.imshow("R2000 raw Image", image)

        # self.error_pub.publish(self.Pose)

        self.Key_Control()


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

    
    def drawAxis(self, img):

        axispts, jacobian = cv2.projectPoints(self.Axis, self.rvec, self.tvec, self.mtx, self.dist)

        axispts = axispts.astype(int)

        img = cv2.line(img, tuple(axispts[0].ravel()), tuple(axispts[1].ravel()), (0, 0, 255), 3)
        img = cv2.line(img, tuple(axispts[0].ravel()), tuple(axispts[2].ravel()), (0, 255, 0), 3)
        img = cv2.line(img, tuple(axispts[0].ravel()), tuple(axispts[3].ravel()), (255, 0, 0), 3)

        return img
    

    def Sequence_Fix(self, Cur_Box_Points):

        for i in range(4):

            dist = []

            for idx in range(i, 4):

                dist.append(np.linalg.norm(self.Pre_Box_Points[i] - Cur_Box_Points[idx]))

            position = np.argmin(dist)

            temp = copy.deepcopy(Cur_Box_Points[i])

            Cur_Box_Points[i] = Cur_Box_Points[position+i]

            Cur_Box_Points[position+i] = temp

        return Cur_Box_Points
    
    
    def Sorting(self, Cur_Box_Points):
        
        criterion = copy.deepcopy(Cur_Box_Points)

        if criterion[0][0] > criterion[1][0]:

            temp2 = copy.deepcopy(Cur_Box_Points[1])

            Cur_Box_Points[1] = Cur_Box_Points[0]

            Cur_Box_Points[0] = temp2

        if criterion[3][0] > criterion[2][0]:

            temp = copy.deepcopy(Cur_Box_Points[2])

            Cur_Box_Points[2] = Cur_Box_Points[3]

            Cur_Box_Points[3] = temp

        return Cur_Box_Points


    def sat_detection(self, origin):

        image = copy.deepcopy(origin)

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        ret, th_img = cv2.threshold(blur, 20, 255, cv2.THRESH_BINARY_INV)

        cv2.imshow("R2000 th Image", th_img)

        edges = cv2.Canny(th_img ,100, 200)

        cv2.imshow("R2000 Edge Image", edges)

        contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        self.cur = timeit.default_timer()

        self.fps = 1.0 / max((self.cur - self.pre), 0.000001)

        self.pre = self.cur

        cv2.putText(image, str(self.fps), (400, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

        cv2.putText(image, str(self.Mode), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

        if contours:

            try:

                cnt = contours[0]

                rect = cv2.minAreaRect(cnt) #returns : ( center (x,y), (width, height), angle of rotation )

                box = cv2.boxPoints(rect)

                box = np.intp(box)

                pt1, pt2, pt3, pt4  = (box[1][0], box[1][1]), (box[2][0], box[2][1]), (box[3][0], box[3][1]), (box[0][0], box[0][1])
                
                corners = np.array([pt1, pt2, pt3, pt4], dtype=np.float64)

                corners = self.Sorting(corners)

                if self.Mode == 1:

                    self.Pre_Box_Points = corners

                elif self.Mode == 2:

                    corners = self.Sequence_Fix(corners)

                    self.Pre_Box_Points = corners

                ret2, temp_rvec, temp_tvec = cv2.solvePnP(self.objs, corners, self.mtx, self.dist)

                self.rvec = np.array([temp_rvec[0][0], temp_rvec[1][0], temp_rvec[2][0]])

                self.tvec = np.array([temp_tvec[0][0], temp_tvec[1][0], temp_tvec[2][0]])

                image = self.drawAxis(image)

                cv2.drawContours(image,[box],0,(0,0,255),2)

                str_Position = "Position X=%0.4f Y=%0.4f Z=%0.4f" % (self.tvec[0], self.tvec[1], self.tvec[2])

                cv2.putText(image, str_Position, (10, 60), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

                R_ct = np.matrix(cv2.Rodrigues(self.rvec)[0])

                R_tc = R_ct.T

                Pitch, Yaw, Roll = self.rotationMat2EulerAngle(R_tc * self.R_flip)

                str_Attitude = "Attitude R=%0.4f P=%0.4f Y=%0.4f" % (math.degrees(Roll), math.degrees(Pitch), math.degrees(Yaw))

                cv2.putText(image, str_Attitude, (10, 90), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

                self.Roll, self.Pitch, self.Yaw = Roll, Pitch, Yaw

                self.Pose = np.array([self.tvec[0], self.tvec[1], self.tvec[2], self.Roll, self.Pitch, self.Yaw], dtype=np.float32)

                cv2.circle(image, (int(rect[0][0]), int(rect[0][1])), 5, (0,0,255), -1)

                for idx in range(4):

                    cv2.circle(image, (int(corners[idx][0]), int(corners[idx][1])), 5, (0,0,255), -1)

                    cv2.putText(image, str(idx+1), (int(corners[idx][0]), int(corners[idx][1])), cv2.FONT_HERSHEY_DUPLEX, 1, (0, 0, 255), 1, cv2.LINE_AA)

                cv2.circle(image, (int(640/2), int(480/2)), 5, (0,255,0), -1)

            except rospy.ROSInterruptException as Exception:

                print(Exception)
                pass

        return image, th_img


if __name__ == '__main__':
    
    try:
      
        rospy.init_node('R2000_relative_position', anonymous=True)

        R2000_SAT_Detector = Sat_Detector()

        rospy.spin()

    except rospy.ROSInterruptException as Exception:
        
        print(Exception)
        pass
