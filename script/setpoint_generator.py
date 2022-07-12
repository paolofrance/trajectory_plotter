#!/usr/bin/env python3
# license removed for brevity
import copy
# import moveit_commander
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose


class current_pos_monitor:

    def __init__(self):
        self.current_pose = Pose()

    def current_callback(self, data):
        self.current_pose = data.pose


def traj_publisher():
    rospy.init_node('traj_publisher')

    nominal_traj_topic = rospy.get_param('nominal_traj_topic')
    human_traj_topic   = rospy.get_param('human_traj_topic'  )
    current_traj_topic = rospy.get_param('current_traj_topic')
    rospy.sleep(.1)

    cpm = current_pos_monitor()
    current_sub = rospy.Subscriber(current_traj_topic, PoseStamped, cpm.current_callback)

    nom_pub = rospy.Publisher(nominal_traj_topic, PoseStamped, queue_size=10)
    hum_pub = rospy.Publisher(human_traj_topic, PoseStamped, queue_size=10)
    rate = 125.0
    ros_rate = rospy.Rate(rate)
    set_point = 0.3
    human_set_point = 0.6
    rospy.sleep(1)

    reference_pose = copy.deepcopy(cpm.current_pose)
    human_reference_pose = copy.deepcopy(cpm.current_pose)
    initial_pose = copy.deepcopy(cpm.current_pose)
    reference_pose.position.x = initial_pose.position.x + set_point
    # reference_pose.position.y = initial_pose.position.y + set_point
    # reference_pose.position.z = initial_pose.position.z + set_point
    human_reference_pose.position.x = initial_pose.position.x + human_set_point

    reference_pose_msg = PoseStamped()
    human_pose_msg = PoseStamped()

    while not rospy.is_shutdown():

        # if abs(cpm.current_pose.position.x-reference_pose.position.x) < 0.005:
        #     set_point = -set_point
        #     reference_pose.position.x = initial_pose.position.x + set_point
        #     print("change")

        # if abs(cpm.current_pose.position.z - reference_pose.position.z) < 0.001:
        #     set_point = -set_point
        #     reference_pose.position.z = initial_pose.position.z + set_point
        #     print("change")

        stamp = rospy.Time.now()

        reference_pose_msg.pose = reference_pose
        reference_pose_msg.header.stamp = stamp
        nom_pub.publish(reference_pose_msg)

        human_pose_msg.pose = human_reference_pose
        human_pose_msg.header.stamp = stamp
        hum_pub.publish(human_pose_msg)

        ros_rate.sleep()


if __name__ == '__main__':
    traj_publisher()
