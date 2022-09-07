#!/usr/bin/env python3
# license removed for brevity

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
# from geometry_msgs.msg import PoseStamped, Pose


def traj_publisher():
    rospy.init_node('fake_traj_publisher')
    rospy.sleep(.1)

    current_traj_topic = rospy.get_param('current_traj_topic')
    print(current_traj_topic)

    cur_pub = rospy.Publisher(current_traj_topic, PoseStamped, queue_size=10)

    rate = 125.0
    ros_rate = rospy.Rate(rate)
    t = 0.0
    rospy.sleep(5)

    while not rospy.is_shutdown():
        # print("rospy is not shutdown")
        t += 1.0/rate
        cur_pose_msg = PoseStamped()

        cur_pose_msg.pose.position.x = 0
        cur_pose_msg.pose.position.y = 0.1
        cur_pose_msg.pose.position.z = 0.15

        stamp = rospy.Time.now()
        cur_pose_msg.header.stamp = stamp

        cur_pub.publish(cur_pose_msg)

        ros_rate.sleep()


if __name__ == '__main__':
    traj_publisher()