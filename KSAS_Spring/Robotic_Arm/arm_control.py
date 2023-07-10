#!/usr/bin/env python

import sympy
import rospy
import numpy as np 
import math as m
import sys
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from sympy import cos, sin, pi
from rospy.exceptions import ROSInterruptException
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState

class r2000_control:
    def __init__(self):

        #Init ros node.
        rospy.init_node("r2000_control", anonymous=True)

        self.relative_coord_sub = rospy.Subscriber('/sat_pose', numpy_msg(Floats), self.PE_callback)

        self.dx, self.dy, self.dz, self.droll, self.dpitch, self.dyaw,  = 0, 0, 0, 0, 0, 0
        
        # Set up subscribers.
        # self.th1_sub = rospy.Subscriber("/r2000_description/joint_1_pc/state", JointControllerState, self.joint_1_cb, queue_size=1)
        # self.th2_sub = rospy.Subscriber("/r2000_description/joint_2_pc/state", JointControllerState, self.joint_2_cb, queue_size=1)
        # self.th3_sub = rospy.Subscriber("/r2000_description/joint_3_pc/state", JointControllerState, self.joint_3_cb, queue_size=1)
        # self.th4_sub = rospy.Subscriber("/r2000_description/joint_4_pc/state", JointControllerState, self.joint_4_cb, queue_size=1)
        # self.th5_sub = rospy.Subscriber("/r2000_description/joint_5_pc/state", JointControllerState, self.joint_5_cb, queue_size=1)
        # self.th6_sub = rospy.Subscriber("/r2000_description/joint_6_pc/state", JointControllerState, self.joint_6_cb, queue_size=1)

        # Set up publishers.
        self.th1_pub = rospy.Publisher("/r2000_description/joint_1_pc/command", Float64, queue_size=1)
        self.th2_pub = rospy.Publisher("/r2000_description/joint_2_pc/command", Float64, queue_size=1)
        self.th3_pub = rospy.Publisher("/r2000_description/joint_3_pc/command", Float64, queue_size=1)
        self.th4_pub = rospy.Publisher("/r2000_description/joint_4_pc/command", Float64, queue_size=1)
        self.th5_pub = rospy.Publisher("/r2000_description/joint_5_pc/command", Float64, queue_size=1)
        self.th6_pub = rospy.Publisher("/r2000_description/joint_6_pc/command", Float64, queue_size=1)

        self.joint_1_goal, self.joint_2_goal, self.joint_3_goal, self.joint_4_goal, self.joint_5_goal, self.joint_6_goal  = 0, 0, 0, 0, 0, 0

        self.joint_1_cur,  self.joint_2_cur,  self.joint_3_cur,  self.joint_4_cur, self.joint_5_cur, self.joint_6_cur = 0, 0, 0, 0, 0, 0

        self.publisher(0,0,0,0,0,0)

        #Puma 560 DH parameters.
        a2, a3, d3, d4 = 0.4318, 0.0203, 0.1500, 0.4318
        self.a = [0, 0, a2, a3, 0, 0]
        self.d = [0, 0, d3, d4, 0, 0]

        # self.Toriginefector, self.b, self.T36 = self.sym_fwd_kinematics()


    def Error_Vel(self):

        self.joint_3_goal = -self.dy

        self.joint_1_goal = -self.dx

        # self.publisher(self.joint_1_goal,0,self.joint_3_goal,0,0,0)

        # self.th1_pub.publish(self.joint_1_goal)
        # self.th3_pub.publish(self.joint_3_goal)

        # print(self.joint_3_cur)

    


    def PE_callback(self, data):

        self.dx, self.dy, self.dz  = data.data[0], data.data[1], data.data[2]

        self.droll, self.dpitch, self.dyaw = data.data[3], data.data[4], data.data[5]
        
        self.Error_Vel()

        self.publisher(0,0,0,0,0,0)



    
    # def joint_1_cb(self, data):
        # self.joint_1_cur = data.data[0]

    # def joint_2_cb(self, data):
        # self.joint_2_cur = data.data[0]

    # def joint_3_cb(self, data):
        # self.joint_3_cur = data.data[0]

    # def joint_4_cb(self, data):
        # self.joint_4_cur = data.data[0]

    # def joint_5_cb(self, data):
        # self.joint_5_cur = data.data[0]

    # def joint_6_cb(self, data):
        # self.joint_6_cur = data.data[0]


    def sym_fwd_kinematics(self):
        i = range(6)        #Number of links.
        T = []
        Toriginefector = 1

        thi, alj, aj, di = sympy.symbols("thi, alj, aj, di")

        Tij = sympy.Matrix([[cos(thi),          -sin(thi),          0,        aj], 
                            [sin(thi)*cos(alj), cos(thi)*cos(alj), -sin(alj), -sin(alj)*di], 
                            [sin(thi)*sin(alj), cos(thi)*sin(alj), cos(alj),  cos(alj)*di],
                            [0,                 0,                 0,         1      ]])

        #Puma 560 DH parameters.
        th1, th2, th3, th4, th5, th6, a2, a3, d3, d4 = sympy.symbols("th1, th2, th3, th4, th5, th6, a2, a3, d3, d4")
        al = [0, -pi/2, 0, -pi/2, pi/2, -pi/2]
        a = [0, 0, a2, a3, 0, 0]
        d = [0, 0, d3, d4, 0, 0]
        th = [th1, th2, th3, th4, th5, th6]

        for j in i:
            Tx = Tij.subs([(thi, th[j]), (alj, al[j]), (aj, a[j]), (di, d[j])])
            T.append(Tx)

        #Direct kinematics.
        for j in i:
            Toriginefector = Toriginefector*T[j]

        Toriginefector = sympy.simplify(Toriginefector)

        r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz = sympy.symbols("r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz")
        Tsym = sympy.Matrix([[r11, r12, r13, px],
                            [r21, r22, r23, py],
                            [r31, r32, r33, pz],
                            [0, 0, 0, 1]])

        T03inv = sympy.simplify((T[0]*T[1]*T[2]).inv())
        b = T03inv*Tsym
        T36 = sympy.simplify(sympy.simplify(T03inv*Toriginefector))

        return Toriginefector, b, T36

    def publisher(self, th1, th2, th3, th4, th5, th6):

        th1_m, th2_m, th3_m, th4_m, th5_m, th6_m = Float64(th1), Float64(th2), Float64(th3), Float64(th4), Float64(th5), Float64(th6)
        self.th1_pub.publish(th1_m)
        self.th2_pub.publish(th2_m)
        self.th3_pub.publish(th3_m)
        self.th4_pub.publish(th4_m)
        self.th5_pub.publish(th5_m)
        self.th6_pub.publish(th6_m)

    def inv_kinematics(self, Px, Py, Pz):
        a2, a3, d3, d4 = self.a[2], self.a[3], self.d[2], self.d[3]

        th1 = m.atan2(Py, Px) - m.atan2(d3, m.sqrt(Px**2 + Py**2 - d3**2))
        
        k = (Px**2 + Py**2 + Pz**2 - a2**2 - a3**2 - d3**2 - d4**2)/(2*a2)
        th3 = m.atan2(a3, d4) - m.atan2(k, m.sqrt(a3**2 + d4**2 - k**2))

        th2 = m.atan2((Px*m.cos(th1) + Py*m.sin(th1))*(a2*m.sin(th3) - d4) - Pz*(a3 + a2*m.cos(th3)),
            (a3 + a2*m.cos(th3))*(Px*m.cos(th1) + Py*m.sin(th1)) + (a2*m.sin(th3) - d4)*Pz) - th3
        
        r13 = self.Toriginefector.row(0).column(2)
        r23 = self.Toriginefector.row(1).column(2)
        r33 = self.Toriginefector.row(2).column(2)

        th4 = m.atan2()


if __name__ == "__main__":

    try:
        # rospy.init_node('R2000_Joint_Command', anonymous=True)
        r2000_c = r2000_control()
        rospy.spin()

        # while not rospy.is_shutdown():

            # p560_c.publisher(-3.4,-1.5,1.5,1.5,1.5)
            # lower="-3.15" upper="3.15"
            # lower="-1.0472" upper="1.32645"
            # lower="-2.30383" upper="3.14159"
            # lower="-6.28319" upper="6.28319"
            # lower="-2.18166" upper="2.18166"
            # lower="-6.28" upper="6.28"
            # r2000_c.publisher(0, 0, 0, 0, 0, 0)
            # r2000_c.publisher(3.0, 1.0, 1.0, -1.0, 1.0, 0.5)
    except (ROSInterruptException):
        sys.exit()



# j1 : -3.1416 ~ 3.1416
# j2 : -1.5707 ~ 1.5707
# j3 : -1.5707 ~ 1.5707
# j4 : -1.5707 ~ 1.5707
# j5 : -1.5707 ~ 1.5707
