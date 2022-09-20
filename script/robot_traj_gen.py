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
    current_traj_topic = rospy.get_param('current_traj_topic')
    current_sub = rospy.Subscriber(current_traj_topic, PoseStamped, cpm.current_callback)

    nom_pub = rospy.Publisher(nominal_traj_topic, PoseStamped, queue_size=10)

    rate = 125.0
    ros_rate = rospy.Rate(rate)
    t = 0.0
    omega = 2 * np.pi * 0.1

    x5 = 0.7              # end

    vel_case_x_2 = 0.06

    rospy.sleep(2)

    robot_reference_pose = PoseStamped()
    robot_reference_pose.pose = copy.deepcopy(cpm.current_pose)
    human_reference_pose = PoseStamped()
    human_reference_pose.pose = copy.deepcopy(cpm.current_pose)
    initial_pose   = copy.deepcopy(cpm.current_pose)

    print(initial_pose)

    while not rospy.is_shutdown():

        t += 1.0/rate
        if abs(robot_reference_pose.pose.position.x - initial_pose.position.x -x5) > 0.001 :
            robot_reference_pose.pose.position.x      = initial_pose.position.x + (vel_case_x_2 * t)
        else:
            robot_reference_pose.pose.position.x = initial_pose.position.x + x5

        robot_reference_pose.pose.position.y    = initial_pose.position.y
        robot_reference_pose.pose.position.z    = initial_pose.position.z

        stamp = rospy.Time.now()

        robot_reference_pose.header.stamp = stamp
        nom_pub.publish(robot_reference_pose)

        br = tf.TransformBroadcaster()
        br.sendTransform((robot_reference_pose.pose.position.x,robot_reference_pose.pose.position.y,robot_reference_pose.pose.position.z),
                         (robot_reference_pose.pose.orientation.x,robot_reference_pose.pose.orientation.y,robot_reference_pose.pose.orientation.z,robot_reference_pose.pose.orientation.w),
                         rospy.Time.now(),
                         "target_pose",
                         "base_link")

        ros_rate.sleep()


if __name__ == '__main__':

    import os
    duration = 10  # seconds
    freq = 440  # Hz

    # os.system('spd-say "ciao ciao"')
    os.system('play -n synth %s sin %s' % (duration, freq))
    # traj_publisher()
