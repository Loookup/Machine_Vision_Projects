#! /usr/bin/env python3

import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension
from geometry_msgs.msg import Twist
import time
from matplotlib import pyplot as plt


class Scout_Mini:

    def __init__(self, _kp, _ki, _kd):

        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.kp = _kp
        self.ki = _ki
        self.kd = _kd
        self.del_t = 0.066  # 1 / FPS
        self.ess_z = 0.3  # 30cm
        self.ess_x = 0.05  # 5cm
        self.min_vel = 0.001  # 1e-4
        self.max_vel = 1.0
        self.error_x_pre = 0
        self.error_x_cur = 0
        self.error_y_pre = 0
        self.error_y_cur = 0
        self.error_z_pre = 0
        self.error_z_cur = 0
        self.cmd_vel = Twist()
        self.height = False
        self.arrival = False
        self.mode = None
        self.error_roll_cur = 0
        self.error_roll_pre = 0
        self.error_pitch_cur = 0
        self.error_pitch_pre = 0
        self.error_yaw_cur = 0
        self.error_yaw_pre = 0
        self.error_tilt_cur = 0
        self.error_tilt_pre = 0
        self.integral_repo_x = 0
        self.integral_repo_y = 0
        self.integral_repo_z = 0
        self.integral_repo_roll = 0
        self.integral_repo_pitch_cam = 0
        self.integral_repo_pitch_body = 0
        self.integral_repo_yaw = 0
        self.goal_x = 0.05
        self.goal_y = 0.3
        self.goal_z = 2.0
        self.mtx = np.array([[631.546, 0, 335.197], [0, 632.597, 243.093], [0, 0, 1]])
        self.distort_mtx = np.array([[0.0, 0.0, 0.0, 0.0]]) # k1, k2, p1, p2, k3
        self.time_graph = []
        self.timer = 0
        self.x_graph = []
        self.y_graph = []
        self.z_graph = []
        self.r_graph = []
        self.p_graph = []
        self.yaw_graph = []
        self.sequence = 0
        self.traj_start = False
        self.traj_aligned = False
        self.survey_end = False
        self.end = False


    def draw_graph(self):

        plt.subplot(3,2,1)
        plt.title("Position X")
        plt.xlabel("Time [sec]")
        plt.ylabel("X [m]")
        plt.plot(self.time_graph, self.x_graph,c='r')
        plt.legend(['X'],loc='upper right')
    
        plt.subplot(3,2,3)
        plt.title("Position Y")
        plt.xlabel("Time [sec]")
        plt.ylabel("Y [m]")
        plt.plot(self.time_graph, self.y_graph,c='g')
        plt.legend(['Y'],loc='upper right')
    
        plt.subplot(3,2,5)
        plt.title("Position Z")
        plt.xlabel("Time [sec]")
        plt.ylabel("Z [m]")
        plt.plot(self.time_graph, self.z_graph,c='b')
        plt.legend(['Z'],loc='upper right')
    
        plt.subplot(3,2,2)
        plt.title("Angular Roll")
        plt.xlabel("Time [sec]")
        plt.ylabel("Radian")
        plt.plot(self.time_graph, self.r_graph, c='r')
        plt.legend(['R'],loc='upper right')
    
        plt.subplot(3,2,4)
        plt.title("Angular Pitch")
        plt.xlabel("Time [sec]")
        plt.ylabel("Radian")
        plt.plot(self.time_graph, self.p_graph, c='g')
        plt.legend(['P'],loc='upper right')
    
        plt.subplot(3,2,6)
        plt.title("Angular Yaw")
        plt.xlabel("Time [sec]")
        plt.ylabel("Radian")
        plt.plot(self.time_graph, self.yaw_graph, c='b')
        plt.legend(['Y'],loc='upper right')
    
        plt.tight_layout()
        plt.savefig("fig1.png", dpi=1500)
        plt.show()


    def Determine_Controller(self, error_cur, error_pre, integral_reposit, goal):

        Proportional_Controller = self.kp * (error_cur - goal)

        Derivative_Controller = self.kd * (error_cur - error_pre) / self.del_t

        if integral_reposit != None:

            Integral_Element = self.ki * (error_cur + error_pre - (2 * goal)) * self.del_t / 2

            integral_reposit = integral_reposit + Integral_Element

        else:

            integral_reposit = 0

        Controller = Proportional_Controller + integral_reposit + Derivative_Controller

        return Controller


    def Output_Velocity(self, error_cur, error_pre, integral_reposit, goal):

        Controller = self.Determine_Controller(error_cur, error_pre, integral_reposit, goal)

        Velocity = Controller * error_cur

        if abs(Velocity) < self.min_vel:

            Velocity = 0

        elif abs(Velocity) >= self.max_vel:  # Saturation

            if Velocity >= 0:
                Velocity = self.max_vel
            else:
                Velocity = -1 * self.max_vel

        return Velocity
    

    def save_cur_to_pre(self):

        self.error_roll_pre = self.error_roll_cur
        self.error_pitch_pre = self.error_pitch_cur
        self.error_yaw_pre = self.error_yaw_cur
        self.error_x_pre, self.error_y_pre, self.error_z_pre = self.error_x_cur, self.error_y_cur, self.error_z_cur
        

    def Arrival_Check(self):

        if abs(self.error_x_cur) < self.goal_x and abs(self.error_y_cur) < self.goal_y and abs(self.error_z_cur) < self.goal_z + 0.05:

            rospy.loginfo("Arrived")
            
            self.arrival = True

            if abs(self.error_y_cur) < 0.4:

                rospy.loginfo("configure")

                self.height = True

            # else:
                # self.height = False

    def reset_pre(self):

        self.error_x_pre = 0
        self.error_y_pre = 0
        self.error_z_pre = 0

        self.error_roll_pre = 0
        self.error_pitch_pre = 0
        self.error_yaw_pre = 0


    def draw(self):

        if self.mode == 1:

            self.time_graph.append(self.timer)
            self.x_graph.append(self.error_x_cur)
            self.y_graph.append(self.error_y_cur)
            self.z_graph.append(self.error_z_cur)
            self.r_graph.append(self.error_roll_cur)
            self.p_graph.append(self.error_pitch_cur)
            self.yaw_graph.append(self.error_yaw_cur)

            self.timer += self.del_t


    def height_pitch(self):

        self.cmd_vel.linear.y = self.Output_Velocity(self.error_y_cur, self.error_y_pre, self.integral_repo_pitch_body, 0)

        self.cmd_vel.linear.z = self.Output_Velocity(self.error_pitch_cur, self.error_pitch_pre, self.integral_repo_pitch_cam, 0)

        if self.error_pitch_cur <= 0:
            self.cmd_vel.linear.z = -1 * abs(self.cmd_vel.linear.z)

        if self.error_y_cur >= 0:

            self.cmd_vel.linear.y = abs(self.cmd_vel.linear.y)

        else:

            self.cmd_vel.linear.y = -1 * abs(self.cmd_vel.linear.y)

        # self.cmd_vel.linear.z = -1 * self.cmd_vel.linear.y + self.cmd_vel.linear.z


    def Turn_Around_Algorithm(self):

        if abs(self.error_yaw_cur) <= 0.1 and abs(self.error_x_cur) <= 0.02:

            self.reset_pre()

            self.sequence = 4
            self.traj_aligned = True
            rospy.loginfo("----Traj Aligned !----")

        rospy.loginfo(self.sequence)

        if self.sequence == 0:

            if abs(self.error_x_cur) > 2.0 or abs(self.error_yaw_cur) <= 0.1:

                self.sequence = 1

                self.reset_pre()

            else:

                # self.cmd_vel.angular.z = self.Output_Velocity(self.error_yaw_cur, self.error_yaw_pre)
                if self.error_yaw_cur < 0:

                    self.cmd_vel.angular.z = -0.7
                    # self.cmd_vel.angular.z = -1 * abs(self.cmd_vel.angular.z)
                else:

                    self.cmd_vel.angular.z = 0.7
                    # self.cmd_vel.angular.z = abs(self.cmd_vel.angular.z)

        elif self.sequence == 1:

            if self.error_z_cur > 5.5:

                self.cmd_vel.linear.x = self.Output_Velocity(self.error_z_cur, self.error_z_pre, self.integral_repo_x, 5.2)

            else:

                self.integral_repo_x = 0
                self.reset_pre()

                self.sequence = 2

        elif self.sequence == 2:

            if abs(self.error_x_cur) <= 0.025:

                self.integral_repo_z = 0

                self.reset_pre()

                self.sequence = 3

                # if abs(self.error_x_cur) < 0.05:
# 
                    # self.sequence = 3
# 
                # else:
                    # self.cmd_vel.angular.z = 0.7




                # else:

                    # self.cmd_vel.angular.z = 0.5

            else:  

                self.cmd_vel.angular.z = self.Output_Velocity(self.error_x_cur, self.error_x_pre, self.integral_repo_z , 0)

                if self.error_x_cur >= 0:

                    self.cmd_vel.angular.z = -1 * self.cmd_vel.angular.z
        
        elif self.sequence == 3:

            if self.error_z_cur < 8.9:

                self.cmd_vel.linear.x = self.Output_Velocity(self.error_z_cur, self.error_z_pre, self.integral_repo_x, 9.2)

                # self.cmd_vel.linear.x = -1 * self.Output_Velocity(self.error_z_cur, self.error_z_pre)

            else:

                self.integral_repo_x = 0

                self.reset_pre()

                self.sequence = 0


    def Survey(self):

        if abs(self.error_x_cur) < 0.03 and self.mode == 1:

            rospy.loginfo("Survey End, Approach")

            self.integral_repo_x = 0

            self.survey_end = True

        else:

            self.survey_end = False

        if self.survey_end == False and self.mode == 0:

            self.cmd_vel.angular.z = -1.0
            self.draw()
            self.cmd_vel_pub.publish(self.cmd_vel)

        elif self.survey_end == False and self.mode == 1:

            self.cmd_vel.angular.z = self.Output_Velocity(self.error_x_cur, self.error_x_pre, self.integral_repo_x, 0)
            if self.error_x_cur >= 0:
                self.cmd_vel.angular.z = -1 * self.cmd_vel.angular.z
        
        elif self.survey_end == True and self.mode == 1:

            if self.error_z_cur > 8.2:

                self.cmd_vel.linear.x = self.Output_Velocity(self.error_z_cur, self.error_z_pre, self.integral_repo_x, 7.9)

            else:

                rospy.loginfo(self.error_z_cur)

                self.integral_repo_x = 0

                self.traj_start = True

                self.reset_pre()

                rospy.loginfo("Traj Start")

        else:

            print("CC")




    def Command(self, data):

        # double wrist_vel = msg->angular.x Roll;
        # double pelvis_vel = msg->angular.y Yaw;
        # double elbow_vel = msg->linear.z Pitch_camera;
        # double hip_vel =  msg->linear.y Pitch_body;

        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.linear.y = 0.0
        self.cmd_vel.linear.z = 0.0  # fixed
        self.cmd_vel.angular.x = 0.0
        self.cmd_vel.angular.y = 0.0  # fixed
        self.cmd_vel.angular.z = 0.0

        self.mode, self.error_x_cur, self.error_y_cur, self.error_z_cur = data.data[0], data.data[1], data.data[2], data.data[3]

        self.error_roll_cur, self.error_pitch_cur, self.error_yaw_cur = data.data[4], data.data[5], data.data[6]

        if abs(self.error_x_cur) <= 0.05 and abs(self.error_y_cur) <= 0.05 and self.traj_aligned == True:

            self.end = True

            rospy.loginfo("----Mission Complete, End----")

        self.draw()

        if self.mode == 2:

            print("--end--")
            self.draw_graph()

            while():
                exit()

        if self.traj_start == False:

            self.Survey()

        else:
            
            if self.traj_aligned == False:

                self.Turn_Around_Algorithm()


        if self.traj_aligned == True and self.end == False:

            self.Arrival_Check()

            self.cmd_vel.angular.x = self.Output_Velocity(self.error_roll_cur, self.error_roll_pre, self.integral_repo_roll, 0)

            
            if self.error_roll_cur <= 0:

                self.cmd_vel.angular.x = -1 * abs(self.cmd_vel.angular.x)

            self.cmd_vel.angular.z = self.Output_Velocity(self.error_x_cur, self.error_x_pre, self.integral_repo_x, 0)
            if self.error_x_cur >= 0:
                self.cmd_vel.angular.z = -1 * self.cmd_vel.angular.z


            if self.arrival == False:

                self.cmd_vel.linear.x = self.Output_Velocity(self.error_z_cur, self.error_z_pre, None, self.goal_z)
 
                # self.cmd_vel.linear.x = self.Output_Velocity(self.error_z_cur, self.error_z_pre)
 
 
            else:

                self.cmd_vel.linear.x = self.Output_Velocity(self.error_x_cur, self.error_x_pre, self.integral_repo_x, 0)
 
                self.cmd_vel.angular.y = self.Output_Velocity(self.error_yaw_cur, self.error_yaw_pre, self.integral_repo_yaw, 0)
 
                if self.error_yaw_cur >= 0:

                    self.cmd_vel.angular.y = -1 * abs(self.cmd_vel.angular.y)
 
                if self.height == False:
    
                    self.cmd_vel.linear.z = self.Output_Velocity(self.error_pitch_cur, self.error_pitch_pre, self.integral_repo_pitch_cam, 0)

                    if self.error_pitch_cur <= 0:
                        self.cmd_vel.linear.z = -1 * abs(self.cmd_vel.linear.z)

                else:
 
                    self.height_pitch()

        self.save_cur_to_pre()

        self.cmd_vel_pub.publish(self.cmd_vel)

        
def callback(data):

    my_Scout_Mini.Command(data)


rospy.init_node('Generate_Command', anonymous=True)

my_Scout_Mini = Scout_Mini(15, 7, 3)

rospy.Subscriber('System_Controller', Float64MultiArray, callback)

if __name__ == '__main__':

    rospy.spin()
