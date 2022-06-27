#!/usr/bin/env python

from __future__ import print_function
from six.moves import input

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, tau, dist, fabs, cos
from tf.transformations import quaternion_from_euler
import tf
import actionlib
import simple_touch_controller_msgs.msg
from configuration_msgs.srv import StartConfiguration, StartConfigurationRequest
from moveit_commander.conversions import pose_to_list


def all_close(goal, actual, tolerance):
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
    x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
    # Euclidean distance
    d = dist((x1, y1, z1), (x0, y0, z0))
    # phi = angle between orientations
    cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
    return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

  return True


class MoveGroupPythonInterfaceTutorial(object):

  def __init__(self):
    super(MoveGroupPythonInterfaceTutorial, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    display_trajectory_publisher = rospy.Publisher(
      "/move_group/display_planned_path",
      moveit_msgs.msg.DisplayTrajectory,
      queue_size=20,
    )

    self.box_name = ""
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.eef_link = move_group.get_end_effector_link()

  def go_to_joint_state(self,joint_goal):
    move_group = self.move_group

    print("planning")
    move_group.set_joint_value_target(joint_goal)
    [em,plan,pt,mm] = move_group.plan()
    rospy.sleep(1)

    move_group.execute(plan, wait=True)

    move_group.stop()

    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


def go_home():
  robot_interface = MoveGroupPythonInterfaceTutorial()
  rospy.wait_for_service('configuration_manager/start_configuration')
  start_config_srv = rospy.ServiceProxy('configuration_manager/start_configuration', StartConfiguration)
  config_request = StartConfigurationRequest()

  config_request.start_configuration = 'planner'
  config_request.strictness = 1
  resp = start_config_srv(config_request)
  # robot_interface.go_to_joint_state([-1.7291563192950647, -1.0304821173297327, 1.4205517768859863, -1.981112305318014, -1.574482266102926, 2.79579758644104])
  # robot_interface.go_to_joint_state([-2.007465664540426, -0.9841740767108362, 1.346189022064209, -1.956766430531637, -1.5702841917621058, 2.5176148414611816]) # ioc
  robot_interface.go_to_joint_state([-1.276287857686178, -0.21938974062074834, 1.5005650520324707, -2.858126942311422, -1.554249111806051, 4.048782825469971]) # load - Z
  # robot_interface.go_to_joint_state([-1.3910163084613245, -0.4043777624713343, 1.766099452972412, -3.1378849188434046, 0.013868370093405247, 2.702698230743408]) #load storto

if __name__ == '__main__':
    print("qqqqqiii")
    go_home()