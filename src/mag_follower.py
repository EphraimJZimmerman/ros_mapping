#!/usr/bin/env python3

import math
import rospy
import numpy
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu, MagneticField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from pid import PID
from tf.transformations import euler_from_quaternion


TOLERANCE_AGAINST_ERROR = 0.0001
TURN_ANGLE_TOLERANCE = 0.01
MIN_TURN_SPEED = 0.2
FORWARD_SPEED = 1
MAGNETIC_DECLINATION_DEGREES = 13.98

class ImuFollower:
    def __init__(self):
        self.cur_yaw = None
        self.pid = PID(-0.6,0.6,1,0.1,5) 
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.bearing_sub = rospy.Subscriber('/robot/bearing', Float64, self.bearing_cb)
        self.turn_angle_sub = rospy.Subscriber('/robot/turn_angle', Float64, self.turn_angle_cb)
        self.imu_mag_sub = rospy.Subscriber('imu/mag_corrected', MagneticField, self.imu_cb)
        self.new_callback = False
        self.change_yaw = False
        self.cur_target_yaw = None
        self.prev_target_yaw = -1 #angles should never be negative so functions as a non-initialized value
        #self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_cb)
        
        

    def bearing_cb(self, msg):
        print("callback: " +str(msg.data))
        self.cur_target_yaw = self.convertNegativeAngles(msg.data * (math.pi/180))
        if self.cur_target_yaw != self.prev_target_yaw:
            print("CHANGING YAW!!!")
            self.change_yaw = True
            self.new_callback = True
        self.prev_target_yaw = self.cur_target_yaw

    def turn_angle_cb(self, msg):
        return

    def imu_cb(self, msg):
        # Extract magnetic field components in ENU convention
         # Extract magnetic field components in NED convention
        Bx = msg.magnetic_field.x  # North South component
        By = msg.magnetic_field.y  # East West component

    # Calculate the yaw angle (heading) in radians
    # yaw_radians = math.atan2(Bx, By)
        self.cur_yaw = math.atan2(-By, Bx) - \
            math.radians(MAGNETIC_DECLINATION_DEGREES)

    # Convert radians to degrees and normalize to [0, 360)
        #yaw_degrees = (math.degrees(yaw_radians) + 360) % 360

    def follow_direction(self, target_yaw):
        self.cmd_vel_pub.publish(Twist())
        rate = rospy.Rate(10)
        self.turn_to_heading(target_yaw, 1)
        self.new_callback = False
        while not self.new_callback:
            #rospy.loginfo("following direction")
            twist = Twist()
            twist.linear.x = FORWARD_SPEED

            #get opposite sign of cur_dist because pid takes a subtraction and want to align with desired turn direction.
            cur_dist = -1*self.difference_from_target(self.cur_yaw, target_yaw)
            twist.angular.z = self.pid.compute(0, cur_dist)
            #print("Target yaw: ", str(target_yaw))
            self.cmd_vel_pub.publish(twist)
            print("Current dist: ", str(-1*cur_dist))
            print("turn: " + str(twist.angular.z))
            rate.sleep()
            
        
    def turn_to_heading(self, target_yaw, base_vel):
        """
        Turns the robot to heading `target_yaw` with a base velocity of
        `base_vel`.
        """
        # if target_yaw < 0.05:
        #     target_yaw = 2*math.pi #helps it slow down at the appropriate time.
        rate = rospy.Rate(20)
        while self.cur_yaw == None or self.cur_target_yaw == None:
            continue #waiting for new odometry values
            rate.sleep()
        
        target_yaw = self.convertNegativeAngles(target_yaw)
        headingTwist = Twist()

        #positive is counterclockwise turn, negative is clockwise turn

        prev_dist = self.difference_from_target(self.cur_yaw, target_yaw)
        cur_dist = self.difference_from_target(self.cur_yaw, target_yaw)

        #while not math.isclose(self.cur_yaw, target_yaw, abs_tol=TURN_ANGLE_TOLERANCE):
        while not self.completed_rotation(prev_dist, cur_dist):
            rospy.loginfo("current yaw:" + str(self.cur_yaw))
            rospy.loginfo("target yaw: " + str(target_yaw))
            rospy.loginfo("distance: " + str(cur_dist))
            #slows rotation as the target is approached, with a minimum and maximum rotation speed defined.
            if abs(cur_dist < 0.4): #0.5*0.4 = 0.2 = min cmd_vel to get branbot to move
                headingTwist.angular.z = 0.2 * (cur_dist/abs(cur_dist))
            else:
                headingTwist.angular.z = 0.5 * cur_dist #add a more balanced turning method?
            self.cmd_vel_pub.publish(headingTwist)
            while self.cur_yaw == None:
                continue #waiting for new odometry values
            prev_dist = cur_dist
            cur_dist = self.difference_from_target(self.cur_yaw, target_yaw)
            rate.sleep()
        rospy.loginfo("current yaw:" + str(self.cur_yaw))
        rospy.loginfo("target yaw: " + str(target_yaw))
        rospy.loginfo("distance: " + str(cur_dist))

       
        self.cmd_vel_pub.publish(Twist()) #a 0-rotation twist to stop movement
    
    def convertNegativeAngles(self, angle):
        '''Converts from [-pi,pi] to [0,2pi]'''
        if angle >= 0:
            return angle
        else:
            return 2*math.pi + angle 

    def difference_from_target(self, cur_yaw, target_yaw):
        '''finds shortest turning amount to get to target(avoid PID freaking out from wrapping from 6.14 to 0 or vice versa)'''
        if cur_yaw <= target_yaw:
            right_turn = -(target_yaw - cur_yaw)
            left_turn = (cur_yaw + (2*math.pi - target_yaw))
        elif cur_yaw > target_yaw:
            left_turn = -(target_yaw - cur_yaw)
            right_turn = -(target_yaw + (2*math.pi - cur_yaw))  
        if abs(right_turn) <= abs(left_turn):
            return right_turn
        else:
            return left_turn
    
    #FIX LATER
    #doesn't work because of noise if on the boundary of closest distance being positive vs negative
    def completed_rotation(self, prev_dist, cur_dist):
        #allowing 180 degree turns
        if math.isclose(abs(self.cur_yaw - self.target_yaw), math.pi, abs_tol=0.15):
            return False
        elif prev_dist <=0 and cur_dist >=0:
            return True
        elif prev_dist >=0 and cur_dist <=0:
            return True
        else:
            return False

    def run(self):
        rate = rospy.Rate(10)
        while self.cur_target_yaw == None:
            continue #waiting for new yaw values
            rate.sleep()
        while not rospy.is_shutdown():
            if self.change_yaw:
                self.change_yaw = False
                self.follow_direction(self.cur_target_yaw)

        
        # cur_target_yaw = self.target_yaw
        # prev_target_yaw = 0
        # if cur_target_yaw != prev_target_yaw:
        #     self.follow_direction(cur_target_yaw)

            

if __name__ == '__main__':
    rospy.init_node('imu_follower')
    ImuFollower().run()
