#!/usr/bin/env python
# license removed for brevity
import random

import numpy as np
import rospy
import matplotlib.pyplot as plt
# import numpy as np
# import math
from geometry_msgs.msg import PoseStamped
import time
# import moveit_commander
import tf
from tf import listener


class Plotter:

    def __init__(self, ax):

        self.ax = ax
        self.show_nom = False
        self.show_cur = False
        self.show_hum = False

        self.x_nom = np.array([])
        self.y_nom = np.array([])

        self.x_init = 0.0
        self.y_init = 0.0
        self.init_pose = True

        self.x_cur = np.array([])
        self.y_cur = np.array([])

        self.x_hum = np.array([])
        self.y_hum = np.array([])

        self.x_0 = 0
        self.y_0 = 0
        self.pose_zeroing = True

        self.circle1 = plt.Circle((0.16, -0.16), 0.03, color='r')

    def nominal_callback(self, data):
        if self.init_pose:
            self.x_init = data.pose.position.x
            self.y_init = data.pose.position.y
            self.init_pose = False
        self.x_nom = data.pose.position.x
        self.y_nom = data.pose.position.y
        self.show_nom = True

    def current_callback(self, data):

        if self.pose_zeroing:
            self.x_0 = data.pose.position.x
            self.y_0 = data.pose.position.y
            self.pose_zeroing = False

        self.x_cur = data.pose.position.x
        self.y_cur = data.pose.position.y

        self.show_cur = True

    def human_callback(self, data):
        self.x_hum = data.pose.position.x
        self.y_hum = data.pose.position.y
        self.show_hum = True

    def add_circle(self, x, y):
        self.circle1 = plt.Circle((x, y), 0.03, color='r')

if __name__ == '__main__':
    rospy.init_node('plotter', anonymous=True)
    nominal_traj_topic = rospy.get_param('nominal_traj_topic')
    current_traj_topic = rospy.get_param('current_traj_topic')
    human_traj_topic   = rospy.get_param('human_traj_topic'  )

    delta_axis = [-0.3, 0.3, -0.3, 0.3] # centered
    delta_axis = [0.1, 0.6, 0.3, 0.3] # left

    fig, Ax = plt.subplots(1)

    plt.ion()
    my_plotter = Plotter(Ax)
    nominal_sub = rospy.Subscriber(nominal_traj_topic, PoseStamped, my_plotter.nominal_callback)
    current_sub = rospy.Subscriber(current_traj_topic, PoseStamped, my_plotter.current_callback)
    human_sub   = rospy.Subscriber(human_traj_topic  , PoseStamped, my_plotter.human_callback  )

    plt.ion()

    rate = rospy.Rate(125)

    while not rospy.is_shutdown():
        if my_plotter.show_nom:
            # if random.uniform(0, 1) < 0.01:
            #     circ = plt.Circle((my_plotter.x_nom, my_plotter.y_nom), 0.03, color='r')
            #     my_plotter.ax.add_patch(circ)
            my_plotter.show_nom = False
            my_plotter.ax.plot(my_plotter.x_nom, my_plotter.y_nom, 'om')
            plt.draw()
            plt.pause(0.001)

        if my_plotter.show_cur:
            my_plotter.show_cur = False
            my_plotter.ax.plot(my_plotter.x_cur, my_plotter.y_cur, 'og')
            plt.draw()
            plt.pause(0.001)

        if my_plotter.show_hum:
            my_plotter.show_hum = False
            my_plotter.ax.plot(my_plotter.x_hum, my_plotter.y_hum, 'ob')
            plt.draw()
            plt.pause(0.001)

        if my_plotter.x_init-0.02 < my_plotter.x_nom < my_plotter.x_init+0.02 and my_plotter.y_init-0.02 < my_plotter.y_nom < my_plotter.y_init+0.02 :
            my_plotter.ax.clear()
        my_plotter.ax.clear()
        axis = [my_plotter.x_0 - delta_axis[0], my_plotter.x_0 + delta_axis[1], my_plotter.y_0 - delta_axis[2], my_plotter.y_0 + delta_axis[3]]
        plt.axis(axis)
        plt.grid()


        rate.sleep()





