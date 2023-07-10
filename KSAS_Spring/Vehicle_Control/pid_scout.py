#! /usr/bin/env python3.9

import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from geometry_msgs.msg import Twist
import time


class Scout_Mini:

    def __init__(self, _kp, _ki, _kd, _cmd_vel):

        self.kp = _kp
        self.ki = _ki
        self.kd = _kd
        self.del_t = 1 / 30  # 1 / FPS
        self.ess_z = 0.3  # 30cm
        self.ess_x = 0.05  # 8cm
        self.min_vel = 0.0001  # 1e-4
        self.max_vel = 0.5
        self.error_x_pre = 0
        self.error_x_cur = 0
        self.error_z_pre = 0
        self.error_z_cur = 0
        self.alignment = False
        self.cmd_vel = _cmd_vel
        self.initiator = True
        self.arrival = False


    def Determine_Controller(self, error_cur, error_pre):

        Proportional_Controller = self.kp * error_cur

        Integral_Controller = self.ki * (error_cur + error_pre) * self.del_t / 2

        Derivative_Controller = self.kd * (error_cur - error_pre) / self.del_t

        Controller = Proportional_Controller + Integral_Controller + Derivative_Controller

        return Controller


    def Output_Velocity(self, error_cur, error_pre):

        Controller = self.Determine_Controller(error_cur, error_pre)

        Velocity = Controller * error_cur

        if abs(Velocity) <= self.min_vel:

            Velocity = 0

        elif abs(Velocity) > self.max_vel:  # Saturation

            if Velocity >= 0:
                Velocity = self.max_vel
            else:
                Velocity = -1 * self.max_vel

        return Velocity


    def Mode_Select(self):

        self.initiator = False

        if self.error_z_cur <= self.ess_z:
            # Select Mode

            self.arrival = True
            rospy.loginfo("Arrived !")

        else:

            self.arrival = False

            if abs(self.error_x_cur) <= self.ess_x:

                self.alignment = True
                # rospy.loginfo("Aligned")

            else:

                self.alignment = False
                # rospy.loginfo("Not Aligned")


    def Command(self, data):

        self.error_x_cur, self.error_z_cur = data.data[0], data.data[2]

        self.Mode_Select()

        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.linear.y = 0.0
        self.cmd_vel.linear.z = 0.0  # fixed
        self.cmd_vel.angular.x = 0.0
        self.cmd_vel.angular.y = 0.0  # fixed
        self.cmd_vel.angular.z = 0.0

        if self.alignment == False and self.arrival == False:
            # Align-Mode
            self.cmd_vel.angular.z = self.Output_Velocity(self.error_x_cur, self.error_x_pre)

            self.error_z_cur = 0

            if self.error_x_cur >= 0:

                self.cmd_vel.angular.z = -1 * self.cmd_vel.angular.z

            # rospy.loginfo("Ang Z : " + str(self.cmd_vel.angular.z))

        elif self.alignment == True and self.arrival == False:
            # Go-Mode
            self.cmd_vel.linear.x = self.Output_Velocity(self.error_z_cur, self.error_z_pre)

            self.error_x_cur = 0

            # rospy.loginfo("Lin X : " + str(self.cmd_vel.linear.x))

        # else:

            # self.cmd_vel.linear.x = 0.0
            # self.cmd_vel.linear.y = 0.0
            # self.cmd_vel.linear.z = 0.0  # fixed
            # self.cmd_vel.angular.x = 0.0
            # self.cmd_vel.angular.y = 0.0  # fixed
            # self.cmd_vel.angular.z = 0.0


        self.error_x_pre, self.error_z_pre = self.error_x_cur, self.error_z_cur

        cmd_vel_pub.publish(self.cmd_vel)


    def Initiator(self):

        # rospy.loginfo("Detecting Mode Start")

        while self.initiator == True:

            self.cmd_vel.angular.z = 0.3

            cmd_vel_pub.publish(self.cmd_vel)

            # if detecting fail > Stop
        

def callback(data):

    my_Scout_Mini.Command(data)


def Start_Timer():

    idx = 5

    Rate = rospy.Rate(1)

    while not idx == 0 and not rospy.is_shutdown():

        rospy.loginfo("Scout Mini T - %d" %idx)

        idx -= 1

        Rate.sleep()



def scout_mini_control():

    rospy.init_node('pid_scout_mini', anonymous=True)

    Start_Timer()

    rospy.Subscriber('object_pose', numpy_msg(Floats), callback)

    my_Scout_Mini.Initiator()

    rospy.spin()



if __name__ == '__main__':

    cmd_vel_pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)

    cmd_vel = Twist()

    my_Scout_Mini = Scout_Mini(6.7, 0.2, 2.5, cmd_vel) # _kp, _ki, _kd, cmd_vel

    scout_mini_control()
