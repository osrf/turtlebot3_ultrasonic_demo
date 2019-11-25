#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

import threading

import time

class PID:
    def __init__(self, Kp, Td, Ti):
        self.Kp = Kp
        self.Td = Td
        self.Ti = Ti
        self.curr_error = 0
        self.prev_error = 0
        self.sum_error = 0
        self.prev_error_deriv = 0
        self.curr_error_deriv = 0
        self.control = 0

    def update_control(self, current_error, dt, reset_prev=False):
        self.curr_error_deriv = (self.curr_error - self.prev_error) / dt

        #steering angle = P gain + D gain + I gain
        p_gain = self.Kp * self.curr_error

        i_gain = self.sum_error  + self.Ti * self.curr_error * dt
        self.sum_error = i_gain
        d_gain = self.Td * self.curr_error_deriv

        #PID control
        w = p_gain + d_gain + i_gain # = control?
        self.control = w

        # update error
        self.prev_error = self.curr_error
        self.curr_error = current_error
        self.prev_error_deriv = self.curr_error_deriv
        #print("control", self.control)
        return self.control


class WallFollow:
    def __init__( self ):
        self.velpub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self.ultrasonic_center_sub = rospy.Subscriber("ultranic_sensor1", Range, self.ultrasonic_center_callback)
        self.ultrasonic_right_sub = rospy.Subscriber("ultranic_sensor3", Range, self.ultrasonic_right_callback)
        self.ultrasonic_left_sub = rospy.Subscriber("ultranic_sensor2", Range, self.ultrasonic_left_callback)

        self.ultrasonic_center = 0.12
        self.ultrasonic_right = 0.12
        self.ultrasonic_left = 0.12

        t = threading.Thread(target=self.thread_function)
        self.pid = PID(3, 10.0, 0.15)
        self.start = time.clock()

    def thread_function(self):
        rclpy.spin()

    def run(self):
        while not rospy.is_shutdown():
            data = [self.ultrasonic_center,
                    self.ultrasonic_left,
                    self.ultrasonic_right]

            F = data[0] < 0.3
            L = data[1] < 0.3
            R = data[2] < 0.3

            end = time.clock()
            target_left_distance = 0.3
            diff_with_target_distance = (self.ultrasonic_left - target_left_distance)
            angle = self.pid.update_control(diff_with_target_distance, end - self.start)

            self.start = end

            if(not F):
                self.move(0.1, angle, 0)
            else:
                self.move(0.0, -0.3, 0)


    def move(self, lin_vel, ang_vel, dur ):
        cmd1 = Twist()#v=lin_vel, omega=-ang_vel)
        cmd1.linear.x = lin_vel
        cmd1.angular.z = ang_vel

        self.velpub.publish( cmd1 )
        # print(cmd1)
        # rospy.sleep( dur )

    def ultrasonic_center_callback(self, msg):
        self.ultrasonic_center = msg.range

    def ultrasonic_right_callback(self, msg):
        self.ultrasonic_right = msg.range

    def ultrasonic_left_callback(self, msg):
        self.ultrasonic_left = msg.range

if __name__ == '__main__':
    rospy.init_node('wall_follow', anonymous=False)
    WallFollow().run()
