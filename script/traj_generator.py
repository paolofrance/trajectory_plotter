#!/usr/bin/env python3
# license removed for brevity
import copy
# import moveit_commander
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Pose


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
    print(nominal_traj_topic)
    print(human_traj_topic)

    cpm = current_pos_monitor()
    current_sub = rospy.Subscriber(current_traj_topic, PoseStamped, cpm.current_callback)
    rospy.sleep(.1)

    nom_pub = rospy.Publisher(nominal_traj_topic, PoseStamped, queue_size=10)
    hum_pub = rospy.Publisher(human_traj_topic  , PoseStamped, queue_size=10)
    rate = 125.0
    ros_rate = rospy.Rate(rate)
    t = 0.0
    t0 = 0.0

    init_time = True

    rho = 0.3
    omega = 1

    t1 = 3.0
    t2 = 5.0
    t3 = 7.0
    t4 = 9.0
    t5 = 11.0

    x2 = 0.005
    x3 = rho * np.sin(omega*t3)
    x4 = -0.01
    x5 = rho * np.sin(omega*t5)

    rospy.sleep(5)

    reference_pose = copy.deepcopy(cpm.current_pose)
    initial_pose = copy.deepcopy(cpm.current_pose)

    while not rospy.is_shutdown():
        t += 1.0/rate
        reference_pose.position.x = initial_pose.position.x + rho * np.sin(omega*t)
        nom_pose_msg = PoseStamped()
        nom_pose_msg.pose = reference_pose

        stamp = rospy.Time.now()
        nom_pose_msg.header.stamp = stamp

        nom_pub.publish(nom_pose_msg)

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
