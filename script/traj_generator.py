#!/usr/bin/env python
# license removed for brevity
import copy
# import moveit_commander
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped


def traj_publisher():
    rospy.init_node('traj_publisher')

    nominal_traj_topic = rospy.get_param('nominal_traj_topic')
    nominal_traj_topic_twist = rospy.get_param('nominal_traj_topic_twist')
    human_traj_topic   = rospy.get_param('human_traj_topic'  )
    print(nominal_traj_topic)
    print(human_traj_topic)
    nom_pub = rospy.Publisher(nominal_traj_topic, PoseStamped, queue_size=10)
    nom_twist_pub = rospy.Publisher(nominal_traj_topic_twist, TwistStamped, queue_size=10)
    hum_pub = rospy.Publisher(human_traj_topic  , PoseStamped, queue_size=10)
    rate = 125.0
    ros_rate = rospy.Rate(rate)
    t = 0.0
    t0 = 0.0

    init_time = True

    rho = 0.2
    omega = 0.5

    t1 = 3.0
    t2 = 5.0
    t3 = 7.0
    t4 = 9.0
    t5 = 11.0

    x2 = 0.005
    x3 = rho * np.sin(omega*t3)
    x4 = -0.01
    x5 = rho * np.sin(omega*t5)

    # move_group = moveit_commander.MoveGroupCommander("ur5_lab")
    # pose_0 = move_group.get_current_pose().pose

    velocity_impedance = False

    rospy.loginfo("press enter")
    # input('press enter')
    rospy.sleep(5)

    while not rospy.is_shutdown():
        t += 1.0/rate

        nom_pose_msg = PoseStamped()
        nom_pose_msg.pose.position.x = rho - rho * np.cos(omega*t)
        # nom_pose_msg.pose.position.y = + rho * np.sin(omega*t)
        nom_pose_msg.pose.position.y = 0
        nom_pose_msg.pose.position.z = 0

        stamp = rospy.Time.now()
        nom_pose_msg.header.stamp = stamp

        nom_pub.publish(nom_pose_msg)

        nom_twist_msg = TwistStamped()
        nom_twist_msg.twist.linear.x = omega * rho * np.sin(omega*t)
        nom_twist_msg.twist.linear.y = omega * rho * np.cos(omega*t)
        nom_twist_msg.twist.linear.z = 0
        nom_twist_msg.header.frame_id = "base"

        nom_twist_pub.publish(nom_twist_msg)

        hum_pose_msg = PoseStamped()
        if t1 < t < t2:
            if init_time:
                t0 = copy.deepcopy(t)
                init_time = False

            x1 = rho * np.sin(omega*t0)

            hum_pose_msg.pose.position.x = rho - rho * np.cos(omega*t)
            hum_pose_msg.pose.position.y = x1 + (x2 - x1) * (t-t0) / (t2-t1)
            hum_pose_msg.pose.position.z = 0

        elif t2 < t < t3:
            if init_time:
                t0 = copy.deepcopy(t)
                init_time = False

            hum_pose_msg.pose.position.x = rho - rho * np.cos(omega*t)
            hum_pose_msg.pose.position.y = x2 + (x3 - x2) * (t-t2) / (t3-t2)
            hum_pose_msg.pose.position.z = 0

        else:
            hum_pose_msg.pose.position.x = rho - rho * np.cos(omega*t)
            hum_pose_msg.pose.position.y =  rho * np.sin(omega*t)
            hum_pose_msg.pose.position.z = 0
            init_time = True

        hum_pose_msg.header.stamp = stamp

        # hum_pub.publish(hum_pose_msg)

        ros_rate.sleep()

        if -0.01 < nom_pose_msg.pose.position.y < 0.0 and 0.0 < nom_pose_msg.pose.position.x < 0.01:
        # if -0.01 < nom_pose_msg.pose.position.y < 0.0 and 2*rho - 0.02 < nom_pose_msg.pose.position.x < 2*rho:
            t = 0.0
            print("end of the turn!")


if __name__ == '__main__':
    traj_publisher()
