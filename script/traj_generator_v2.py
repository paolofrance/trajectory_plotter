#!/usr/bin/env python3
# license removed for brevity
import copy
# import moveit_commander
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Pose
import tf

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
    # hum_pub = rospy.Publisher(human_traj_topic  , PoseStamped, queue_size=10)
    rate = 125.0
    ros_rate = rospy.Rate(rate)
    t = 0.0
    # t0 = 0.0
    # init_time = True

    # PARAM FOR CREATING THE TRAJECTORY
    omega = 2 * np.pi * 0.1

    x0 = 0              # start
    x1 = 0.05            # begin deviation
    x2 = 0.2            # end deviation
    x3 = 0.40           # begin to return on nom. traj
    x4 = 0.55            # reach the nom traj
    x5 = 0.8              # end

    vel_case_x_2 = 0.06
    amplit_dev   = 0.2

    rospy.sleep(2)

    robot_reference_pose = PoseStamped()
    robot_reference_pose.pose = copy.deepcopy(cpm.current_pose)
    human_reference_pose = PoseStamped()
    human_reference_pose.pose = copy.deepcopy(cpm.current_pose)
    initial_pose   = copy.deepcopy(cpm.current_pose)

    print(initial_pose)

    # CASES FOR X
    case_x = 2
    # 1: cosine
    # 2: constant velocity,

    while not rospy.is_shutdown():
        t += 1.0/rate

        # X (both human and nom):
        if case_x == 1:             # X as cosine
            human_reference_pose.pose.position.x = initial_pose.position.x + 0.5 * x5 * (1 - np.cos(omega * t))
            robot_reference_pose.pose.position.x = initial_pose.position.x + 0.5 * x5 * (1 - np.cos(omega * t))
        elif case_x == 2:           # X as constant velocity
            if ((vel_case_x_2 * t) // x5) % 2 == 0:    # even after floor
                human_reference_pose.pose.position.x = initial_pose.position.x + (vel_case_x_2 * t) % x5
                robot_reference_pose.pose.position.x      = initial_pose.position.x + (vel_case_x_2 * t) % x5
            else:
                human_reference_pose.pose.position.x = initial_pose.position.x + x5 - (vel_case_x_2 * t) % x5
                robot_reference_pose.pose.position.x      = initial_pose.position.x + x5 - (vel_case_x_2 * t) % x5

        if robot_reference_pose.pose.position.x - initial_pose.position.x < x1:
            robot_reference_pose.pose.position.z = initial_pose.position.z
        elif x1 <= robot_reference_pose.pose.position.x - initial_pose.position.x < x2:
            robot_reference_pose.pose.position.z = initial_pose.position.z + amplit_dev * 0.5 * \
                                           (1 - np.cos(np.pi * (robot_reference_pose.pose.position.x - x1) / (x2 - x1)))
        elif x2 <= robot_reference_pose.pose.position.x - initial_pose.position.x < x3:
            robot_reference_pose.pose.position.z = initial_pose.position.z + amplit_dev
        elif x3 <= robot_reference_pose.pose.position.x - initial_pose.position.x < x4:
            robot_reference_pose.pose.position.z = initial_pose.position.z + amplit_dev * 0.5 * \
                                           (1 + np.cos(np.pi * (robot_reference_pose.pose.position.x - x3) / (x4 - x3)))
        else:
            robot_reference_pose.pose.position.z = initial_pose.position.z

        # REFERENCE POSITION y-z
        robot_reference_pose.pose.position.y    = initial_pose.position.y
        human_reference_pose.pose.position.y    = initial_pose.position.y
        human_reference_pose.pose.position.z    = initial_pose.position.z

        stamp = rospy.Time.now()

        human_reference_pose.header.stamp = stamp
        # hum_pub.publish(human_reference_pose)

        robot_reference_pose.header.stamp = stamp
        nom_pub.publish(robot_reference_pose)

        # if robot_reference_pose.pose.position.x - initial_pose.position.x - x5 <= 0.001:
        #     rospy.loginfo("ended ! stop")
        #     rospy.sleep(10)
        br = tf.TransformBroadcaster()
        br.sendTransform(robot_reference_pose.pose.position,
                         robot_reference_pose.pose.orientation,
                         rospy.Time.now(),
                         "target_pose",
                         "base_link")

        ros_rate.sleep()


if __name__ == '__main__':
    traj_publisher()
