#! /usr/bin/env python3

import copy
import numpy as np
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
        #init_state_msgs = rospy.wait_for_message(
        #    '/ag_gripper/joint_states', JointState)
        #self.q_zero = np.array(init_state_msgs.position)
        # self.q_zero = np.array([0.5, -1.0, -2.0, 1.0])
        self.open_pos = np.array([-1.0, 0.075])
        self.closed_pos = np.array([0.5, 2.0])
        self.motor_names = ['mx28', 'mx64']
        self.duration_open = 1.0
        self.duration_close = 0.6
        # close gripper and cutter msg
        self.close_msg = JointTrajectory()
        self.close_msg.joint_names = self.motor_names
        close_jtp = JointTrajectoryPoint()
        close_jtp.positions = self.closed_pos
        close_jtp.time_from_start = rospy.Duration.from_sec(self.duration_close)
        self.close_msg.points.append(close_jtp)

        # open gripper and cutter msg
        self.open_msg = JointTrajectory()
        self.open_msg.joint_names = self.motor_names
        open_jtp = JointTrajectoryPoint()
        open_jtp.positions = self.open_pos
        open_jtp.time_from_start = rospy.Duration.from_sec(self.duration_open)
        self.open_msg.points.append(open_jtp)

        # close and open cutter individually during harvesting procedure
        self.close_cutter_harvest_msg = JointTrajectory()
        self.close_cutter_harvest_msg.joint_names = self.motor_names
        close_cutter_harvest_jtp = JointTrajectoryPoint()
        close_cutter_harvest_jtp.positions = self.closed_pos
        close_cutter_harvest_jtp.time_from_start = rospy.Duration.from_sec(self.duration_close)
        self.close_cutter_harvest_msg.points.append(close_cutter_harvest_jtp)
        
        self.open_cutter_harvest_msg = JointTrajectory()
        self.open_cutter_harvest_msg.joint_names = self.motor_names
        open_cutter_harvest_jtp = JointTrajectoryPoint()
        open_cutter_harvest_jtp.positions = np.array([self.closed_pos[0], self.open_pos[1]])
        open_cutter_harvest_jtp.time_from_start = rospy.Duration.from_sec(self.duration_open)  
        self.open_cutter_harvest_msg.points.append(open_cutter_harvest_jtp)
    

        # start service
        self.service = rospy.Service('/gripper_service', Pegasus, self.run)


        # subscriber to joy

        self.controller_sub = rospy.Subscriber('/joy', Bool, self.cb_gripper_actuate)
        self.buttons = None

        rospy.spin()
        
        
      # gripper acuate cb



    def cb_gripper_actuate(self, msg):
        self.buttons = msg.buttons
        
    def run(self, req):
        if req.command == 0:
            #harvest procedure
            rospy.loginfo("------ HARVESTING PROCEDURE INITIATED ------")
            rospy.loginfo("Opening gripper and cutter....")
            self.controller_pub.publish(self.open_msg)
            rospy.sleep(3.0)
            
            rospy.loginfo("Closing gripper....")
            q = rospy.wait_for_message(
            '/ag_gripper/joint_states', JointState)
    
            self.close_gripper_msg = JointTrajectory()
            self.close_gripper_msg.joint_names = self.motor_names
            close_gripper_jtp = JointTrajectoryPoint()
            close_gripper_jtp.positions = np.array([self.closed_pos[0], q.position[1]])
            close_gripper_jtp.time_from_start = rospy.Duration.from_sec(1.0)
            self.close_gripper_msg.points.append(close_gripper_jtp)
            self.controller_pub.publish(self.close_gripper_msg)
            
            rospy.sleep(3.0)
            rospy.loginfo("Cutting 3x....")
            self.controller_pub.publish(self.close_cutter_harvest_msg)
            rospy.sleep(1.0)
            self.controller_pub.publish(self.open_cutter_harvest_msg)
            rospy.sleep(3.0)
            self.controller_pub.publish(self.close_cutter_harvest_msg)
            rospy.sleep(1.0)
            self.controller_pub.publish(self.open_cutter_harvest_msg)
            rospy.sleep(3.0)
            self.controller_pub.publish(self.close_cutter_harvest_msg)
            rospy.sleep(1.0)
            self.controller_pub.publish(self.open_cutter_harvest_msg)
            rospy.sleep(3.0)
            rospy.loginfo("------ HARVESTING PROCEDURE COMPLETED ------")
            
            return PegasusResponse(1)
        
        elif req.command == 1:
            #open everything
            rospy.loginfo("Opening gripper and cutter....")
            self.controller_pub.publish(self.open_msg)
            #rospy.sleep(3.0)

            return PegasusResponse(1)
        

        elif req.command == 2:
            #close everything
            rospy.loginfo("Closing gripper and cutter....")
            self.controller_pub.publish(self.close_msg)
            #rospy.sleep(3.0)

            return PegasusResponse(1)
        
        elif req.command == 3:
            #open cutter
            rospy.loginfo("Opening cutter....")
            q = rospy.wait_for_message(
            '/ag_gripper/joint_states', JointState)

            self.open_cutter_msg = JointTrajectory()
            self.open_cutter_msg.joint_names = self.motor_names
            open_cutter_jtp = JointTrajectoryPoint()
            open_cutter_jtp.positions = np.array([q.position[0], self.open_pos[1]])
            open_cutter_jtp.time_from_start = rospy.Duration.from_sec(self.duration_open)
            self.open_cutter_msg.points.append(open_cutter_jtp)

            self.controller_pub.publish(self.open_cutter_msg)

            return PegasusResponse(1)
        
        elif req.command == 4:
            #close cutter
            rospy.loginfo("Closing cutter....")
            q = rospy.wait_for_message(
            '/ag_gripper/joint_states', JointState)

            # open and close cutter individually
            self.close_cutter_msg = JointTrajectory()
            self.close_cutter_msg.joint_names = self.motor_names
            close_cutter_jtp = JointTrajectoryPoint()
            close_cutter_jtp.positions = np.array([q.position[0], self.closed_pos[1]])
            close_cutter_jtp.time_from_start = rospy.Duration.from_sec(self.duration_close)
            self.close_cutter_msg.points.append(close_cutter_jtp)

            self.controller_pub.publish(self.close_cutter_msg)

            return PegasusResponse(1)
        
        elif req.command == 5:
            #close cutter
            rospy.loginfo("Opening gripper....")
            q = rospy.wait_for_message(
            '/ag_gripper/joint_states', JointState)

            self.open_gripper_msg = JointTrajectory()
            self.open_gripper_msg.joint_names = self.motor_names
            open_gripper_jtp = JointTrajectoryPoint()
            open_gripper_jtp.positions = [self.open_pos[0], q.position[1]]
            open_gripper_jtp.time_from_start = rospy.Duration.from_sec(self.duration_open)
            self.open_gripper_msg.points.append(open_gripper_jtp)
            self.controller_pub.publish(self.open_gripper_msg)

            return PegasusResponse(1)
        
        elif req.command == 6:
            #close cutter
            rospy.loginfo("Closing gripper....")
            q = rospy.wait_for_message(
            '/ag_gripper/joint_states', JointState)
    
            self.close_gripper_msg = JointTrajectory()
            self.close_gripper_msg.joint_names = self.motor_names
            close_gripper_jtp = JointTrajectoryPoint()
            close_gripper_jtp.positions = np.array([self.closed_pos[0], q.position[1]])
            close_gripper_jtp.time_from_start = rospy.Duration.from_sec(self.duration_close)
            self.close_gripper_msg.points.append(close_gripper_jtp)
            self.controller_pub.publish(self.close_gripper_msg)

            return PegasusResponse(1)
        
        elif req.command == 7:
            rospy.loginfo("------ CALIBRATION PROCEDURE INITIATED ------")
            
            return PegasusResponse(1)
        
        
        # elif req.command == 3:
        #         #teleop mode, depreciated (mapping buttons to independent service calls instead)
        #         rospy.loginfo("Waiting for user input....")

        #         current_time = rospy.get_time()
        #         while rospy.get_time() - current_time < 20.0:
            
        #             if self.buttons[5] == 1:
        #                 self.controller_pub.publish(self.open_msg)
        #             if self.buttons[4] == 1:
        #                 self.controller_pub.publish(self.close_msg)

                

                


        else:
            rospy.loginfo("INCORRECT INPUT, NO ACTION TAKEN")

            return PegasusResponse(0)
                


            # rospy.sleep(5.0)

    # def gripper_cut(self):
    #     self.controller_pub.publish(self.cut_msg)
    #     rospy.sleep(1.0)


if __name__ == "__main__":
    rospy.init_node("gripper_driver")
    rospy.loginfo("gripper driver started")
    motor_driver_wrapper = MotorDriverROSWrapper()
    
