#!/usr/bin/env python3
# license removed for brevity
import copy
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Pose
import os
import signal
import subprocess
import rospkg


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
    current_sub        = rospy.Subscriber(current_traj_topic  , PoseStamped, cpm.current_callback)

    print('\n PUBLISHED TOPICS:')
    print(nominal_traj_topic)
    print(human_traj_topic)

    nom_pub  = rospy.Publisher(nominal_traj_topic, PoseStamped, queue_size = 10)
    hum_pub  = rospy.Publisher(human_traj_topic  , PoseStamped, queue_size = 10)
    rate     = 125.0
    ros_rate = rospy.Rate(rate)
    t        = 0.0

    # PARAM FOR CREATING THE TRAJECTORY
    cos_freq     = 0.05
    omega        = 2 * np.pi * cos_freq
    vel_case_x_2 = 0.05

    x0 = 0              # start
    x1 = 0.1            # begin deviation
    x2 = 0.25           # end deviation
    x3 = 0.30           # begin to return on nom. traj
    x4 = 0.45           # reach the nom traj
    x5 = 0.5            # end

    wait = 3

    rospy.sleep(1)

    reference_pose = copy.deepcopy(cpm.current_pose)
    initial_pose   = copy.deepcopy(cpm.current_pose)

    print('\n INITIAL POSE: ')
    print(str(initial_pose) + '\n')

    # CASES FOR X
    case_x = 1
    # 1: cosine
    # 2: constant velocity,

    # Run a bash file containing 'rosbag record -a'
    record = input("Do you want to record? y/n \n")
    if record == 'y' or record == 'Y':
        PATH = rospkg.RosPack().get_path('trajectory_plotter')
        run = "0"
        record_process = subprocess.Popen(PATH + "/script/bag_record.sh", start_new_session=True,
                                          stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
        print("Acquisition of bag record, DON'T QUIT \n")

    while run != "stop" and (not rospy.is_shutdown()):
        t += 1.0/rate

        # HUMAN POSE
        hum_pose_msg = PoseStamped()
        # X (both human and nom):
        if case_x == 1:             # X as cosine
            if t < wait:   # wait 3 seconds
                hum_pose_msg.pose.position.x = initial_pose.position.x
                reference_pose.position.x    = initial_pose.position.x
            elif (t-wait) <= 1/cos_freq:
                hum_pose_msg.pose.position.x = initial_pose.position.x + 0.5 * x5 * (1 - np.cos(omega * (t - wait)))
                reference_pose.position.x    = initial_pose.position.x + 0.5 * x5 * (1 - np.cos(omega * (t - wait)))
            else:
                os.killpg(os.getpgid(record_process.pid), signal.SIGTERM)
                run = input('Press enter to continue, else digit "stop" \n')
                if run != "stop":
                    t = 0
                    if record == 'y' or record == 'Y':
                        record_process = subprocess.Popen(PATH + "/script/bag_record.sh", start_new_session=True,
                                                          stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
                        print("Acquisition of bag record, DON'T QUIT \n")
        elif case_x == 2:           # X as constant velocity
            if t < wait:
                if ((vel_case_x_2 * t) // x5) % 2 == 0:  # even after floor
                    hum_pose_msg.pose.position.x = initial_pose.position.x
                    reference_pose.position.x    = initial_pose.position.x
                else:
                    hum_pose_msg.pose.position.x = initial_pose.position.x
                    reference_pose.position.x    = initial_pose.position.x
            elif (t-wait) <= (2 * x5) / vel_case_x_2:
                if ((vel_case_x_2 * t) // x5) % 2 == 0:    # even after floor
                    hum_pose_msg.pose.position.x = initial_pose.position.x + (vel_case_x_2 * (t - wait)) % x5
                    reference_pose.position.x    = initial_pose.position.x + (vel_case_x_2 * (t - wait)) % x5
                else:
                    hum_pose_msg.pose.position.x = initial_pose.position.x + x5 - (vel_case_x_2 * (t - wait)) % x5
                    reference_pose.position.x    = initial_pose.position.x + x5 - (vel_case_x_2 * (t - wait)) % x5
            else:
                os.killpg(os.getpgid(record_process.pid), signal.SIGTERM)
                run = input('Press enter to continue, else digit "stop" \n')
                if run != "stop":
                    t = 0
                    if record == 'y' or record == 'Y':
                        record_process = subprocess.Popen(PATH + "/script/bag_record.sh", start_new_session=True,
                                                          stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
                        print("Acquisition of bag record, DON'T QUIT \n")

        # human target Y:
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