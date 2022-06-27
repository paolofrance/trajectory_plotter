#!/usr/bin/env python
# license removed for brevity
import copy
# import moveit_commander
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, WrenchStamped
from geometry_msgs.msg import Pose
import random


class current_pos_monitor:

    def __init__(self):
        self.current_pose = Pose()
        self.wrench = WrenchStamped()

    def current_callback(self, data):
        self.current_pose = data.pose

    def wrench_callback(self, data):
        self.wrench = data


def traj_publisher():
    rospy.init_node('traj_publisher')

    nominal_traj_topic = rospy.get_param('nominal_traj_topic')
    current_traj_topic = rospy.get_param('current_traj_topic')
    human_traj_topic = rospy.get_param('human_traj_topic')
    rospy.sleep(.1)

    cpm = current_pos_monitor()
    current_sub = rospy.Subscriber(current_traj_topic, PoseStamped, cpm.current_callback)
    wrench_sub = rospy.Subscriber("filtered_wrench_base", WrenchStamped, cpm.wrench_callback)

    nom_pub = rospy.Publisher(nominal_traj_topic, PoseStamped, queue_size=10)
    hum_pub = rospy.Publisher(human_traj_topic, PoseStamped, queue_size=10)
    rate = 125.0
    ros_rate = rospy.Rate(rate)
    set_point = 0.6
    # set_point = random.uniform(.4, .9)
    # duration = set_point/2
    duration = 0.3
    rospy.sleep(.5)


    reference_pose = copy.deepcopy(cpm.current_pose)
    human_pose = copy.deepcopy(cpm.current_pose)
    initial_pose = copy.deepcopy(cpm.current_pose)
    # reference_pose.position.x = initial_pose.position.x + set_point
    # reference_pose.position.y = initial_pose.position.y + set_point
    # reference_pose.position.z = initial_pose.position.z + set_point

    goal_pose = copy.deepcopy(cpm.current_pose)
    goal_pose.position.z =  initial_pose.position.z + set_point

    reference_pose_msg = PoseStamped()
    human_pose_msg = PoseStamped()

    count =0
    start_setpoint=False

    while not rospy.is_shutdown():

        if abs(cpm.wrench.wrench.force.z) > 0.5:
            start_setpoint = True

        if start_setpoint:
            count = count+1
            if abs(reference_pose.position.z - goal_pose.position.z) > 0.05:
                reference_pose.position.z = initial_pose.position.z + set_point/duration * count/rate
            else:
                reference_pose.position.z = initial_pose.position.z + set_point
        else:
            reference_pose.position.z = initial_pose.position.z

        reference_pose_msg.pose = reference_pose
        stamp = rospy.Time.now()
        reference_pose_msg.header.stamp = stamp
        nom_pub.publish(reference_pose_msg)

        human_pose_msg.pose = goal_pose
        human_pose_msg.header.stamp = stamp
        hum_pub.publish(human_pose_msg)

        ros_rate.sleep()


if __name__ == '__main__':
    traj_publisher()
