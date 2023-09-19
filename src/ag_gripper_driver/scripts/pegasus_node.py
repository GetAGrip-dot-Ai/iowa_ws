#! /usr/bin/env python3

import copy
import numpy as np
import rospy
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class MotorDriverROSWrapper:

    def __init__(self):
        self.gripper_closed = False
        self.controller_pub = rospy.Publisher(
            '/ag_gripper/joint_trajectory', JointTrajectory, queue_size=1)

        # echo joint states and save as zero_offset
        init_state_msgs = rospy.wait_for_message(
            '/ag_gripper/joint_states', JointState)
        self.q_zero = np.array(init_state_msgs.position)
        # self.q_zero = np.array([0.5, -1.0, -2.0, 1.0])

        # close gripper and cutter msg
        self.close_msg = JointTrajectory()
        self.close_msg.joint_names = ['mx28', 'mx64']
        close_jtp = JointTrajectoryPoint()
        close_jtp.positions = np.array([-0.4, 0.2])
        close_jtp.time_from_start = rospy.Duration.from_sec(1.0)
        self.close_msg.points.append(close_jtp)

        # open gripper and cutter msg
        self.open_msg = JointTrajectory()
        self.open_msg.joint_names = ['mx28', 'mx64']
        open_jtp = JointTrajectoryPoint()
        open_jtp.positions = np.array([-1.571, -1.2533])
        open_jtp.time_from_start = rospy.Duration.from_sec(1.0)
        self.open_msg.points.append(open_jtp)

        # close and open cutter individually
        self.close_cutter_msg = JointTrajectory()
        self.close_cutter_msg.joint_names = ['mx28', 'mx64']
        close_cutter_jtp = JointTrajectoryPoint()
        close_cutter_jtp.positions = np.array([-0.4, 0.2])
        close_cutter_jtp.time_from_start = rospy.Duration.from_sec(0.2)
        self.close_cutter_msg.points.append(close_cutter_jtp)
        
        self.open_cutter_msg = JointTrajectory()
        self.open_cutter_msg.joint_names = ['mx28', 'mx64']
        open_cutter_jtp = JointTrajectoryPoint()
        open_cutter_jtp.positions = np.array([-0.4, -1.2533])
        open_cutter_jtp.time_from_start = rospy.Duration.from_sec(0.5)  
        self.open_cutter_msg.points.append(open_cutter_jtp)

        # close gripper individually
        self.close_gripper_msg = JointTrajectory()
        self.close_gripper_msg.joint_names = ['mx28', 'mx64']
        close_gripper_jtp = JointTrajectoryPoint()
        close_gripper_jtp.positions = np.array([-0.4, -1.2533])
        close_gripper_jtp.time_from_start = rospy.Duration.from_sec(1)
        self.close_gripper_msg.points.append(close_gripper_jtp)

        self.run()
   
        
        

        # release_jtp = JointTrajectoryPoint()
        # release_jtp.positions = [self.q_zero[0]]
        # release_jtp.time_from_start = rospy.Duration.from_sec(6.0)
        # self.cut_msg.points.append(cut_jtp)
        # self.cut_msg.points.append(release_jtp)

        # # gripper calibrate cb
        # self.gripper_cut_sub = rospy.Subscriber(
        #     'ag_gripper/gripper_calibrate', Bool, self.cb_gripper_calibrate)

        # # gripper acuate cb
        # self.controller_sub = rospy.Subscriber(
        #     'joy', Bool, self.cb_gripper_actuate)

    # def cb_gripper_actuate(self, msg):
    #     if msg.buttons[5] > 0:
    #         self.gripper_close()
    #     if msg.buttons[4] > 0:
    #         self.gripper_cut()

    def run(self):

        while not rospy.is_shutdown():

            if self.gripper_closed is False:
                self.controller_pub.publish(self.close_gripper_msg)
                self.gripper_closed = True
                rospy.loginfo("Closing gripper....")
                rospy.sleep(3.0)
            
            else:
                self.gripper_closed = False
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
                rospy.loginfo("Opening gripper ....")
                self.controller_pub.publish(self.open_msg)
                rospy.sleep(3.0)
                


            # rospy.sleep(5.0)

    # def gripper_cut(self):
    #     self.controller_pub.publish(self.cut_msg)
    #     rospy.sleep(1.0)

    # def cb_gripper_calibrate(self, msg):
    #     if msg.data is True:
    #         calib_msg = JointTrajectory()
    #         calib_jtp = JointTrajectoryPoint()
    #         calib_msg.points.append(calib_jtp)

    #         for motor_id in range(0, 4):
    #             calib_msg.joint_names = ['motor'+str(motor_id)]
    #             calib_msg.points[0].positions = [self.q_zero[motor_id]]
    #             for i in range(1, 10):
    #                 q_msg = rospy.wait_for_message(
    #                     '/ag_gripper/joint_states', JointState)
    #                 curr_effort = q_msg.effort[motor_id]
    #                 if np.abs(curr_effort) > 60:
    #                     self.q_zero[motor_id] = copy.deepcopy(
    #                         calib_msg.points[0].positions[0])
    #                     break
    #                 if motor_id == 3:
    #                     calib_msg.points[0].positions = [
    #                         self.q_zero[motor_id] - i*0.1]
    #                 else:
    #                     calib_msg.points[0].positions = [
    #                         self.q_zero[motor_id] + i*0.1]
    #                 calib_msg.points[0].time_from_start = rospy.Duration.from_sec(
    #                     1.0)
    #                 self.controller_pub.publish(calib_msg)
    #                 rospy.sleep(1.0)


if __name__ == "__main__":
    rospy.init_node("gripper_driver")
    rospy.loginfo("gripper driver started")
    motor_driver_wrapper = MotorDriverROSWrapper()
    rospy.spin()
