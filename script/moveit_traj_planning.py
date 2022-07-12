#!/usr/bin/env python3

from __future__ import print_function
from six.moves import input

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import copy

class MoveGroupPythonInterface(object):

  def __init__(self):
    super(MoveGroupPythonInterface, self).__init__()

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

  def go_to_pose_goal(self, pose_goal):

    move_group = self.move_group
    move_group.set_pose_target(pose_goal)

    [em,plan,pt,mm] = move_group.plan()
    print("plan ok, input")
    input()
    move_group.execute(plan, wait=True)

    move_group.stop()
    move_group.clear_pose_targets()

  def plan_cartesian_path(self, scale=1):
    group = self.move_group
    waypoints = []

    wpose = group.get_current_pose().pose
    wpose.position.z -= scale * 0.1  # First move up (z)
    wpose.position.y += scale * 0.2  # and sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= scale * 0.1  # Third move sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = group.compute_cartesian_path(
      waypoints,  # waypoints to follow
      0.01,  # eef_step
      0.0)  # jump_threshold
    return plan, fraction

  def display_trajectory(self, plan):
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory)


def main():
  try:
    ur5interface = MoveGroupPythonInterface()

    pub = rospy.Publisher('/target_cart_pose', geometry_msgs.msg.PoseStamped, queue_size=10)

    current_pose = ur5interface.move_group.get_current_pose(ur5interface.move_group.get_end_effector_link())
    print(current_pose)

    pose_goal = copy.deepcopy(current_pose)

    pose_goal.pose.position.y = pose_goal.pose.position.y - 0.1

    # ur5interface.go_to_pose_goal(pose_goal)

    current_pose = ur5interface.move_group.get_current_pose(ur5interface.move_group.get_end_effector_link())
    print(current_pose)

    cartesian_plan, fraction = ur5interface.plan_cartesian_path()

    ur5interface.display_trajectory(cartesian_plan)

    rospy.sleep(5)

    ur5interface.move_group.execute(cartesian_plan, wait=True)

    # while not is_in_tolerance(pose_goal, current_pose, pose_tol):
    #   pub.publish(pose_goal)
    #   current_pose = ur5interface.move_group.get_current_pose(ur5interface.move_group.get_end_effector_link())
    #   r.sleep()
    #
    # print("DONE")
    # rospy.spin()


  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return


if __name__ == "__main__":
  main()