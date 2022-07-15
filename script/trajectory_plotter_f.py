#!/usr/bin/env python3
# license removed for brevity
import numpy as np
import rospy
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped
# from std_msgs.msg import Float32


class Plotter:

    def __init__(self, ax):

        self.ax = ax
        self.show_nom = False
        self.show_cur = False
        self.show_hum = False
        self.show_obst = False

        self.x_nom = np.array([])
        self.y_nom = np.array([])
        self.z_nom = np.array([])

        self.x_init = 0.0
        self.y_init = 0.0
        self.z_init = 0.0
        self.init_pose = True

        self.x_cur = np.array([])
        self.y_cur = np.array([])
        self.z_cur = np.array([])

        self.x_hum = np.array([])
        self.y_hum = np.array([])
        self.z_hum = np.array([])

        self.x_obst = np.array([])
        self.y_obst = np.array([])

        self.x_0 = 0
        self.y_0 = 0
        self.z_0 = 0
        self.pose_zeroing = True

        self.circle1 = plt.Circle((0.16, -0.16), 0.03, color='g')

    def nominal_callback(self, data):
        if self.init_pose:
            rospy.sleep(0.1)
            self.x_init = data.pose.position.x
            self.y_init = data.pose.position.y
            self.z_init = data.pose.position.z
            self.init_pose = False
        self.x_nom = data.pose.position.x
        self.y_nom = data.pose.position.y
        self.z_nom = data.pose.position.z
        self.show_nom = True

    def current_callback(self, data):

        if self.pose_zeroing:
            self.x_0 = data.pose.position.x
            self.y_0 = data.pose.position.y
            self.z_0 = data.pose.position.z
            self.pose_zeroing = False

        self.x_cur = data.pose.position.x
        self.y_cur = data.pose.position.y
        self.z_cur = data.pose.position.z
        self.show_cur = True

    def human_callback(self, data):
        self.x_hum = data.pose.position.x
        self.y_hum = data.pose.position.y
        self.z_hum = data.pose.position.z
        self.show_hum = True

    def obst_callback(self, data):
        self.x_obst = data.pose.position.x
        self.y_obst = data.pose.position.y
        self.show_obst = True

    def add_circle(self, x, y):
        self.circle1 = plt.Circle((x, y), 0.03, color='r')


def human_target(x1, x2, x3, x4, x5, ampl):

    x_human_target = np.arange(0, x5, 0.001)

    y1   = np.full(int(x1 * 1000), 0)
    x_12 = np.linspace(start=x1, stop=x2, num=round((x2 - x1), 2) * 1000, endpoint=False)
    y2   = ampl * 0.5 * (1 - np.cos(np.pi * (x_12 - x1) / (x2 - x1)))
    y3   = np.full(int(round((x3 - x2), 2) * 1000), ampl)
    x_34 = np.linspace(start=x3, stop=x4, num=round((x4 - x3), 2) * 1000, endpoint=False)
    y4   = ampl * 0.5 * (1 + np.cos(np.pi * (x_34 - x3) / (x4 - x3)))
    y5   = np.full(int(round((x5 - x4), 2) * 1000), 0)

    y_human_target = np.concatenate((y1, y2, y3, y4, y5))

    return x_human_target, y_human_target


if __name__ == '__main__':
    X_ht, Y_ht = human_target(x1=0.10, x2=0.25, x3=0.30, x4=0.45, x5=0.50, ampl=0.08)

    start_point = 0
    end_point = 0.5
    rospy.init_node('plotter', anonymous=True)
    nominal_traj_topic = rospy.get_param('nominal_traj_topic')
    current_traj_topic = rospy.get_param('current_traj_topic')
    human_traj_topic   = rospy.get_param('human_traj_topic')
    obst_x_coord_topic = rospy.get_param('obst_x_coord_topic')

    # Define the margin / size of the plot window
    delta_axis = [0.05, 0.05, 0.05, 0.12]  # all view

    fig, Ax = plt.subplots(1)

    plt.ion()
    my_plotter = Plotter(Ax)
    nominal_sub = rospy.Subscriber(nominal_traj_topic, PoseStamped, my_plotter.nominal_callback)
    current_sub = rospy.Subscriber(current_traj_topic, PoseStamped, my_plotter.current_callback)
    human_sub   = rospy.Subscriber(human_traj_topic  , PoseStamped, my_plotter.human_callback)
    obst_sub    = rospy.Subscriber(obst_x_coord_topic, PoseStamped, my_plotter.obst_callback)
    # OBSTACLE
    while my_plotter.pose_zeroing:
        rospy.sleep(0.5)

    free_trajectory = True
    h_line = True
    # obstacle = True
    # if not free_trajectory:
    obst_x, obst_y = my_plotter.x_0 + 0.275, my_plotter.y_0 + 0

    plt.ion()

    rate = rospy.Rate(125)

    while not rospy.is_shutdown():

        if my_plotter.show_obst:
            my_plotter.show_obst = False
            # obst = plt.Circle((my_plotter.x_0 + my_plotter.x_obst,
            #                    my_plotter.y_0 + my_plotter.y_obst), 0.04, color='r')
            obst = plt.Rectangle((my_plotter.x_0 + my_plotter.x_obst/2,
                                  my_plotter.y_0 + my_plotter.y_obst), 0.06, 0.04, color='r')
            my_plotter.ax.add_patch(obst)

        if my_plotter.show_nom:
            # if obstacle:
            #     obst = plt.Circle((obst_x, obst_y), 0.02, color='r')
            #     my_plotter.ax.add_patch(obst)
            my_plotter.show_nom = False
            my_plotter.ax.plot(my_plotter.x_nom, my_plotter.y_nom, 'or', label='Robot Target')

        if my_plotter.show_cur:
            my_plotter.show_cur = False
            my_plotter.ax.plot(my_plotter.x_cur, my_plotter.y_cur, 'Xg', markersize='10', label='Current Pose')

        if my_plotter.show_hum:
            my_plotter.show_hum = False
            my_plotter.ax.plot(my_plotter.x_hum, my_plotter.y_hum, 'ob', label='Human Target')

        plt.draw()
        plt.legend()
        plt.pause(0.001)

        my_plotter.ax.clear()
        axis = [my_plotter.x_0 - delta_axis[0], my_plotter.x_0 + end_point + delta_axis[1],
                my_plotter.y_0 - delta_axis[2], my_plotter.y_0 + delta_axis[3]]
        plt.axis(axis)
        plt.subplots_adjust(left=0.04, bottom=0.07, right=0.98, top=0.88, wspace=0.06, hspace=0.06)
        my_plotter.ax.set_aspect('equal')
        my_plotter.ax.set_axisbelow(True)
        plt.grid()
        plt.plot(np.linspace(my_plotter.x_0, my_plotter.x_0 + end_point, 100),
                 np.full(100, my_plotter.y_0), '--r', linewidth=1.2)   # label='Nominal robot')
        if not free_trajectory:
            plt.plot(X_ht + my_plotter.x_0, Y_ht + my_plotter.y_0, '--b', linewidth=1.2, label='Human target')
        if h_line:
            plt.plot(np.linspace(my_plotter.x_0, my_plotter.x_0 + end_point, 100),
                     np.full(100, my_plotter.y_0 + 0.08), '--b', linewidth=1, label='Stay below this')
        plt.title('TITLE:')

        rate.sleep()
