#! /usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import time


class Robot_Arm:

    def __init__(self, _kp, _ki, _kd):

        self.manipulator_pub = rospy.Publisher('arm_controller/command', JointTrajectory, queue_size=10)
        self.cmd_tra = JointTrajectory()
        self.cmd_vel = JointTrajectoryPoint()
        self.timer = rospy.Time.now()
        self.cmd_tra.header.stamp = self.timer
        self.cmd_tra.joint_names = ["pelvis", "hip", "shoulder", "elbow", "wrist"]
        self.kp = _kp
        self.ki = _ki
        self.kd = _kd
        self.del_t = 2 / 30  # 1 / FPS
        self.ess_z = 0.3  # 30cm
        self.ess_x = 0.05  # 7cm
        self.min_vel = 0.001  # 1e-3
        self.max_vel = 1
        self.error_roll_pre = 0
        self.error_roll_cur = 0
        self.error_pitch_pre = 0
        self.error_pitch_cur = 0
        self.error_yaw_pre = 0
        self.error_yaw_cur = 0
        self.alignment = False
        self.arrival = False
        self.mode = None


    def Determine_Controller(self, error_cur, error_pre):

        Proportional_Controller = self.kp * error_cur

        Integral_Controller = self.ki * (error_cur + error_pre) * self.del_t / 2

        Derivative_Controller = self.kd * (error_cur - error_pre) / self.del_t

        Controller = Proportional_Controller + Integral_Controller + Derivative_Controller

        return Controller


    def Output_Velocity(self, error_cur, error_pre):

        Controller = self.Determine_Controller(error_cur, error_pre)

        Velocity = Controller * error_cur

        if abs(Velocity) < self.min_vel:

            Velocity = 0

        elif abs(Velocity) >= self.max_vel:  # Saturation

            if Velocity >= 0:
                Velocity = self.max_vel
            else:
                Velocity = -1 * self.max_vel

        return Velocity


    def Mode_Select(self):

        self.initiator = False

        # Select Mode
        if self.error_z_cur <= self.ess_z:
            # Only Rotation

            self.arrival = True

            self.alignment = False

        else:

            self.arrival = False

            if abs(self.error_x_cur) <= self.ess_x:

                self.alignment = True

                self.del_t = 1 / 30

            else:

                self.del_t = 2 / 30


    def Command(self, data):

        self.mode, self.error_roll_cur, self.error_pitch_cur, self.error_yaw_cur = data.data[0], data.data[4], data.data[5], data.data[6]

        # print(self.mode)

        # self.Mode_Select()

        self.cmd_vel.velocities = [1, 1, 1, 1, 1]  # Pelvis(Yaw), hip, shoulder, elbow(pich), wrist(roll)

        self.timer = self.timer + rospy.Time.now()

        self.cmd_vel.time_from_start = self.timer

        self.cmd_tra.points.append(self.cmd_vel)

        print(self.cmd_vel.time_from_start)

        # if self.alignment == False:
        
            # self.cmd_vel.angular.z = self.Output_Velocity(self.error_x_cur, self.error_x_pre)
# 
            # if self.error_x_cur >= 0:
# 
                # self.cmd_vel.angular.z = -1 * self.cmd_vel.angular.z
# 
            # self.alignment = True
# 
        # else:
# 
            # if self.arrival == False:

                # self.cmd_vel.linear.x = self.Output_Velocity(self.error_z_cur, self.error_z_pre)
# 
                # self.alignment = False
# 
        # self.error_x_pre, self.error_z_pre = self.error_x_cur, self.error_z_cur
# 
        # print("[Vel Lin X] : " + str(self.cmd_vel.linear.x))
# 
        # print("[Vel Ang Z] : " + str(self.cmd_vel.angular.z))

        self.manipulator_pub.publish(self.cmd_tra)

        

def callback(data):

    my_Robot_Arm.Command(data)



rospy.init_node('Robot_Arm_Controller', anonymous=True)

rospy.Subscriber('System_Controller', Float64MultiArray, callback)

if __name__ == '__main__':

    print(rospy.Time.now())

    my_Robot_Arm = Robot_Arm(10, 3, 5) # _kp, _ki, _kd, cmd_vel

    rospy.spin()
