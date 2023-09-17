#!/usr/bin/env python3
import rospy
import cv2
import copy
import numpy as np
import sys
import moveit_commander
from sensor_msgs.msg import Image
from sensor_msgs.msg import Joy
from ag_gripper_driver.srv import Pegasus
from std_msgs.msg import Int16
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

class VisualServoingNode:
    def __init__(self):

        rospy.init_node('visual_servoing_node', anonymous=True)
        self.rate= rospy.Rate(30)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.joystick_callback = rospy.Subscriber('/joy', Joy, self.joystick_callback,queue_size=1)

        #self.joystick_callback = rospy.Subscriber('/keyboard', Int16, self.joystick_callback)
        self.target_x = rospy.get_param('~target_x', 320)  # Target x-coordinate in the image center
        self.target_y = rospy.get_param('~target_y', 240)  # Target y-coordinate in the image center
        self.k_p = rospy.get_param('~p_gain', 0.001)  # Proportional gain for visual servoing
        self.joy_state = Joy()
  


    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", e)
            return

        # Perform image processing or feature extraction (you can use OpenCV or other libraries)

        # Calculate error between target and actual feature position
        target_error_x = self.target_x - target_actual_x
        target_error_y = self.target_y - target_actual_y

        # Visual Servoing control law
        vel_cmd = Twist()
        vel_cmd.linear.x = self.k_p * target_error_y
        vel_cmd.angular.z = self.k_p * target_error_x

        self.cmd_vel_pub.publish(vel_cmd)


    def joystick_callback(self, data):
        self.joy_state = data
        # check if any of the joystick buttons are pressed
        # if A button is pressed call gripper open service
        if self.joy_state.buttons[0] == 1:
            rospy.wait_for_service('/gripper_service')
            try:
                 cutter = rospy.ServiceProxy('/gripper_service',Pegasus)
                 # pass args to service
                 cutter(5)
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
            return
        # if B button is pressed call gripper close service
        if self.joy_state.buttons[1] == 1:
            rospy.wait_for_service('/gripper_service')
            try:
                 cutter = rospy.ServiceProxy('/gripper_service',Pegasus)
                 # pass args to service
                 cutter(6)
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
            return
        
        



        # if LB button is pressed open cutter service
        if self.joy_state.buttons[4] == 1:
            rospy.wait_for_service('/gripper_service')
            try:
                 cutter = rospy.ServiceProxy('/gripper_service',Pegasus)
                 # pass args to service
                 cutter(3)
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
            return

        # if RB button is pressed close cutter service
        if self.joy_state.buttons[5] == 1:
            rospy.wait_for_service('/gripper_service')
            try:
                 cutter = rospy.ServiceProxy('/gripper_service',Pegasus)
                 # pass args to service
                 cutter(4)
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
            return
                
        
        
  
    def run(self):
        while not rospy.is_shutdown():
 
            self.rate.sleep()


    
if __name__ == '__main__':
    try:
        visual_servoing_node = VisualServoingNode()        
        visual_servoing_node.run()
    except rospy.ROSInterruptException:
        pass

