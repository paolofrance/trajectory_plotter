#!/usr/bin/env python3
# license removed for brevity
import copy
# import moveit_commander
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Pose


class CurrentPoseMonitor:

    def __init__(self):
        self.current_pose = Pose()

    def current_callback(self, data):
        self.current_pose = data.pose


def traj_publisher():
    rospy.init_node('traj_publisher')

    cpm = CurrentPoseMonitor()
    rospy.sleep(.1)

    nominal_traj_topic = rospy.get_param('nominal_traj_topic')
    human_traj_topic   = rospy.get_param('human_traj_topic')
    current_traj_topic = rospy.get_param('current_traj_topic')
    current_sub = rospy.Subscriber(current_traj_topic, PoseStamped, cpm.current_callback)
    print(nominal_traj_topic)
    print(human_traj_topic)

    nom_pub = rospy.Publisher(nominal_traj_topic, PoseStamped, queue_size=10)
    hum_pub = rospy.Publisher(human_traj_topic  , PoseStamped, queue_size=10)
    rate = 125.0
    ros_rate = rospy.Rate(rate)
    t = 0.0
    # t0 = 0.0
    # init_time = True

    # PARAM FOR CREATING THE TRAJECTORY
    omega = 2 * np.pi * 0.05

    x0 = 0              # start
    x1 = 0.15            # begin deviation
    x2 = 0.20            # end deviation
    x3 = 0.30           # begin to return on nom. traj
    x4 = 0.35            # reach the nom traj
    x5 = 0.4              # end

    vel_case_x_2 = 0.03
    amplit_dev   = 0.07

    rospy.sleep(2)

    reference_pose = copy.deepcopy(cpm.current_pose)
    initial_pose   = copy.deepcopy(cpm.current_pose)

    print(initial_pose)

    # CASES FOR X
    case_x = 2
    # 1: cosine
    # 2: constant velocity,
    # 3: slower x_vel when deviation

    # CASES FOR Y:
    case_y = 2
    # 1: tanh
    # 2: cosine

    while not rospy.is_shutdown():
        t += 1.0/rate

        # HUMAN POSE
        hum_pose_msg = PoseStamped()
        # X (both human and nom):
        if case_x == 1:             # X as cosine
            hum_pose_msg.pose.position.x = initial_pose.position.x + 0.5 * x5 * (1 - np.cos(omega * t))
            reference_pose.position.x    = initial_pose.position.x + 0.5 * x5 * (1 - np.cos(omega * t))
        elif case_x == 2:           # X as constant velocity
            if ((vel_case_x_2 * t) // x5) % 2 == 0:    # even after floor
                hum_pose_msg.pose.position.x = initial_pose.position.x + (vel_case_x_2 * t) % x5
                reference_pose.position.x    = initial_pose.position.x + (vel_case_x_2 * t) % x5
            else:
                hum_pose_msg.pose.position.x = initial_pose.position.x + x5 - (vel_case_x_2 * t) % x5
                reference_pose.position.x    = initial_pose.position.x + x5 - (vel_case_x_2 * t) % x5
        # elif case_x == 3:           # slower x_vel when deviation
        #     if ((vel_case_x_2 * t) // x5) % 2 == 0:    # even after floor
        #         hum_pose_msg.pose.position.x = initial_pose.position.x + (vel_case_x_2 * t) % x5
        #         reference_pose.position.x    = initial_pose.position.x + (vel_case_x_2 * t) % x5
        #     else:
        #         hum_pose_msg.pose.position.x = initial_pose.position.x + x5 - (vel_case_x_2 * t) % x5
        #         reference_pose.position.x    = initial_pose.position.x + x5 - (vel_case_x_2 * t) % x5

        # Y:
        if case_y == 1:             # Y deviation as tanh
            if hum_pose_msg.pose.position.x - initial_pose.position.x < x1:
                hum_pose_msg.pose.position.y = initial_pose.position.y
            elif x1 <= hum_pose_msg.pose.position.x - initial_pose.position.x < x2:
                hum_pose_msg.pose.position.y = initial_pose.position.y + amplit_dev * 0.5 * (1 + (
                                (np.tanh((6 * (hum_pose_msg.pose.position.x - x1) / (x2 - x1)) - 3)) / (np.tanh(3))))
            elif x2 <= hum_pose_msg.pose.position.x - initial_pose.position.x < x3:
                hum_pose_msg.pose.position.y = initial_pose.position.y + amplit_dev
            elif x3 <= hum_pose_msg.pose.position.x - initial_pose.position.x < x4:
                hum_pose_msg.pose.position.y = initial_pose.position.y + amplit_dev * 0.5 * (1 - (
                                (np.tanh((6 * (hum_pose_msg.pose.position.x - x3) / (x4 - x3)) - 3)) / (np.tanh(3))))
            else:
                hum_pose_msg.pose.position.y = initial_pose.position.y
        elif case_y == 2:           # Y deviation as cosine
            if hum_pose_msg.pose.position.x - initial_pose.position.x < x1:
                hum_pose_msg.pose.position.y = initial_pose.position.y
            elif x1 <= hum_pose_msg.pose.position.x - initial_pose.position.x < x2:
                hum_pose_msg.pose.position.y = initial_pose.position.y + amplit_dev * 0.5 * \
                                               (1 - np.cos(np.pi * (hum_pose_msg.pose.position.x - x1) / (x2 - x1)))
            elif x2 <= hum_pose_msg.pose.position.x - initial_pose.position.x < x3:
                hum_pose_msg.pose.position.y = initial_pose.position.y + amplit_dev
            elif x3 <= hum_pose_msg.pose.position.x - initial_pose.position.x < x4:
                hum_pose_msg.pose.position.y = initial_pose.position.y + amplit_dev * 0.5 * \
                                               (1 + np.cos(np.pi * (hum_pose_msg.pose.position.x - x3) / (x4 - x3)))
            else:
                hum_pose_msg.pose.position.y = initial_pose.position.y

        # REFERENCE POSITION Y
        reference_pose.position.y    = initial_pose.position.y
        reference_pose.position.z    = initial_pose.position.z
        hum_pose_msg.pose.position.z = initial_pose.position.z

        stamp = rospy.Time.now()
        hum_pose_msg.header.stamp = stamp
        nom_pose_msg              = PoseStamped()
        nom_pose_msg.pose         = reference_pose
        nom_pose_msg.header.stamp = stamp

        hum_pub.publish(hum_pose_msg)
        nom_pub.publish(nom_pose_msg)

        ros_rate.sleep()


if __name__ == '__main__':
    traj_publisher()
