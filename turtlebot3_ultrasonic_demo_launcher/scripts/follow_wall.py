#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

import threading

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
        self.mode = False # True - following, False - picking behavior
        self.wall = -1 #-1 - no wall, 1 - left, 2 - right

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

            # derecha negativo

            target_left_distance = 0.3
            diff_with_target_distance = (self.ultrasonic_left - target_left_distance)*2

            if(not F):
                self.move(0.1, diff_with_target_distance, 0)
            else:
                self.move(0.0, -0.2, 0)


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
