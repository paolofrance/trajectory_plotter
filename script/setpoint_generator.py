#!/usr/bin/env python
# license removed for brevity
import copy
# import moveit_commander
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TwistStamped


class current_pos_monitor:

    def __init__(self):
        self.x_0 = 0
        self.pose_zeroing = True
        self.x_cur = 0

    def current_callback(self, data):
        if self.pose_zeroing:
            # self.x_0 = data.position.x
            self.x_0 = data.pose.position.x
            self.pose_zeroing = False
            print("zeroing"+str(self.x_0))

        # self.x_cur = data.position.x - self.x_0
        self.x_cur = data.pose.position.x - self.x_0

        # print("current: " + str(self.x_cur))


def traj_publisher():
    rospy.init_node('traj_publisher')

    nominal_traj_topic = rospy.get_param('nominal_traj_topic')
    current_traj_topic = rospy.get_param('current_traj_topic')

    cpm = current_pos_monitor()
    # current_sub = rospy.Subscriber(current_traj_topic, PoseStamped, cpm.current_callback)
    current_sub = rospy.Subscriber(current_traj_topic, PoseStamped, cpm.current_callback)

    nom_pub = rospy.Publisher(nominal_traj_topic, PoseStamped, queue_size=10)
    rate = 125.0
    ros_rate = rospy.Rate(rate)
    set_point = 0.3
    rospy.loginfo("wait 5 seconds")
    rospy.sleep(5)

    while not rospy.is_shutdown():

        if abs(cpm.x_cur-set_point) < 0.001:
            set_point = -set_point
            print(abs(cpm.x_cur-set_point))
            print((cpm.x_cur))

        # print("x_cur: "+str(cpm.x_cur))
        # print("set_point: "+str(set_point))
        # print("abs: "+str(abs(cpm.x_cur-set_point)))

        nom_pose_msg = PoseStamped()
        nom_pose_msg.pose.position.x = set_point
        nom_pose_msg.pose.position.y = 0
        nom_pose_msg.pose.position.z = 0

        stamp = rospy.Time.now()
        nom_pose_msg.header.stamp = stamp
        nom_pub.publish(nom_pose_msg)

        ros_rate.sleep()


if __name__ == '__main__':
    traj_publisher()
