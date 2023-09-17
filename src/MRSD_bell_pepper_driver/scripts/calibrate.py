#! /usr/bin/env python3

import copy
import numpy as np
from numpy.linalg import norm
import rospy
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ag_gripper_driver.srv import Pegasus, PegasusResponse

class MotorDriverROSWrapper:

    def __init__(self):
        self.gripper_closed = False
        self.controller_pub = rospy.Publisher(
            '/ag_gripper/joint_trajectory', JointTrajectory, queue_size=1)

        # echo joint states and save as zero_offset
        init_state_msgs = rospy.wait_for_message(
            '/ag_gripper/joint_states', JointState)
        self.q_zero = np.array(init_state_msgs.position)
        self.q = np.zeros(2)
        self.effort = np.zeros(2)

        self.motor_names = ['mx28', 'mx64']

        # gripper calibrate cb
        self.state_sub = rospy.Subscriber(
            'ag_gripper/joint_states', JointState, self.cb_state_sub)
        
        self.calibrate()

    def cb_state_sub(self, msg):
         self.q = np.array(msg.position)
         self.effort = np.array(msg.effort)

    def calibrate(self):
        if norm(self.q) != 0 and norm(self.effort) != 0:
            calib_msg = JointTrajectory()
            calib_jtp = JointTrajectoryPoint()
            calib_msg.points.append(calib_jtp)

            for i in range(len(self.motor_names)):
                calib_msg.joint_names = [self.motor_names[i]]
                calib_msg.points[0].positions = [self.q_zero[i]]
                for j in range(1, 10):
                    q_msg = rospy.wait_for_message(
                        '/ag_gripper/joint_states', JointState)
                    curr_effort = q_msg.effort[i]
                    if np.abs(curr_effort) > 5:
                        self.q_zero[i] = copy.deepcopy(
                            calib_msg.points[0].positions[0]) + [2]
                        break
                    
                    calib_msg.points[0].positions = [self.q_zero[i] - j*0.1]
                    
                    calib_msg.points[0].time_from_start = rospy.Duration.from_sec(
                        1.0)
                    self.controller_pub.publish(calib_msg)
                    rospy.sleep(1.0)


    def cb_gripper_actuate(self, msg):
        self.buttons = msg.buttons
        
    def run(self, req):
        if req.command == 0:

                rospy.loginfo("Opening gripper and cutter....")
                self.controller_pub.publish(self.open_msg)
                rospy.sleep(3.0)
                self.controller_pub.publish(self.close_gripper_msg)
                rospy.loginfo("Closing gripper....")
                rospy.sleep(3.0)
                rospy.loginfo("Cutting 3x....")
                self.controller_pub.publish(self.close_cutter_msg)
                rospy.sleep(1.0)
                self.controller_pub.publish(self.open_cutter_msg)
                rospy.sleep(3.0)
                self.controller_pub.publish(self.close_cutter_msg)
                rospy.sleep(1.0)
                self.controller_pub.publish(self.open_cutter_msg)
                rospy.sleep(3.0)
                self.controller_pub.publish(self.close_cutter_msg)
                rospy.sleep(1.0)
                self.controller_pub.publish(self.open_cutter_msg)
                rospy.sleep(3.0)
                
                return PegasusResponse(1)
        
        elif req.command == 1:
                rospy.loginfo("Opening gripper and cutter....")
                self.controller_pub.publish(self.open_msg)
                rospy.sleep(3.0)

                return PegasusResponse(1)
        

        elif req.command == 2:
                rospy.loginfo("Closing gripper and cutter....")
                self.controller_pub.publish(self.close_msg)
                rospy.sleep(3.0)

                return PegasusResponse(1)
        
        elif req.command == 3:
                
                rospy.loginfo("Waiting for user input....")

                current_time = rospy.get_time()
                while rospy.get_time() - current_time < 20.0:
            
                    if self.buttons[5] == 1:
                        self.controller_pub.publish(self.open_msg)
                    if self.buttons[4] == 1:
                        self.controller_pub.publish(self.close_msg)

                

                return PegasusResponse(1)


        else:
            rospy.loginfo("You entered something else, no action taken")

            return PegasusResponse(0)
                


            # rospy.sleep(5.0)

    # def gripper_cut(self):
    #     self.controller_pub.publish(self.cut_msg)
    #     rospy.sleep(1.0)


if __name__ == "__main__":
    rospy.init_node("gripper_driver")
    rospy.loginfo("gripper driver started")
    motor_driver_wrapper = MotorDriverROSWrapper()
    
