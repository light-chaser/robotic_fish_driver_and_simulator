#!/usr/bin/env python
import rospy
import time
from math import pi, sin, cos, acos
import random
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
"""
Topics To Write on:
type: std_msgs/Float64
/naro/NaroBody_1_NaroBody_2_joint_position_controller/command
/naro/NaroBody_1_Naro_pectoral_fin_left_joint_position_controller/command
/naro/NaroBody_1_Naro_pectoral_fin_right_joint_position_controller/command
/naro/NaroBody_2_NaroBody_3_joint_position_controller/command
/naro/NaroBody_3_Naro_caudal_fin_joint_position_controller/command
"""

class NaroJointMover(object):

    def __init__(self):
        rospy.init_node('naro_jointmover_demo', anonymous=True)
        rospy.loginfo("Naro JointMover Initialising...")

        self.naro_topics = ["/naro/NaroBody_1_NaroBody_2_joint_position_controller/command",
                        "/naro/NaroBody_1_Naro_pectoral_fin_left_joint_position_controller/command",
                        "/naro/NaroBody_1_Naro_pectoral_fin_right_joint_position_controller/command",
                        "/naro/NaroBody_2_NaroBody_3_joint_position_controller/command",
                        "/naro/NaroBody_3_Naro_caudal_fin_joint_position_controller/command"]

        self.publishers_array = []

        i = 0
        for topic in self.naro_topics:
            self.publisher = rospy.Publisher(
                self.naro_topics[i],
                Float64,
                queue_size=1)
            self.publishers_array.append(self.publisher)
            i += 1


        joint_states_topic_name = "/naro/joint_states"
        rospy.Subscriber(joint_states_topic_name, JointState, self.naro_joints_callback)
        naro_joints_data = None
        rate_try = rospy.Rate(1)
        while naro_joints_data is None and not rospy.is_shutdown():
            try:
                naro_joints_data = rospy.wait_for_message(joint_states_topic_name, JointState, timeout=1)
            except:
                rospy.logwarn("Time out " + str(joint_states_topic_name))
                rate_try.sleep()



        self.naro_joint_dictionary = dict(zip(naro_joints_data.name, naro_joints_data.position))
        self.naro_desired_joint_dictionary = dict(zip(naro_joints_data.name, len(naro_joints_data.name)*[0]))



    def naro_joints_callback(self, msg):
        """
        sensor_msgs/JointState
        std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
        string[] name
        float64[] position
        float64[] velocity
        float64[] effort

        :param msg:
        :return:
        """
        self.naro_joint_dictionary = dict(zip(msg.name, msg.position))


    def move_naro_leg_joints(self, position_array):
        """
        NaroBody_1_NaroBody_2_joint, NaroBody_1_Naro_pectoral_fin_left_joint, NaroBody_1_Naro_pectoral_fin_right_joint,
  NaroBody_2_NaroBody_3_joint, NaroBody_3_Naro_caudal_fin_joint
        :param position_dict:
        :return:
        """

        i = 0
        for position in position_array:

            joint_pos = Float64()
            joint_pos.data = position_array[i]
            self.publishers_array[i].publish(joint_pos)
            i += 1


    def naro_swim_sequence(self):
        """

        :return:
        """
        rate = rospy.Rate(20.0)
        i = 0
        delta = 0.01
        max_angle = pi/15.0
        up = True
        while not rospy.is_shutdown():
            sin(i)
            angle_array = [i] * len(self.naro_topics)
            self.move_naro_leg_joints(angle_array)
            rospy.logdebug("Angle="+str(i))
            if up:
                i += delta
            else:
                i -= delta

            if i >= max_angle:
                up = False
            elif i <= -max_angle:
                up = True
            else:
                pass

            rate.sleep()



if __name__ == "__main__":
    naro_jointmover_object = NaroJointMover()
    naro_jointmover_object.naro_swim_sequence()

