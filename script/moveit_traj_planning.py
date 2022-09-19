#!/usr/bin/env python3

from __future__ import print_function

import control_msgs.msg
from six.moves import input

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
from std_msgs.msg import Int64
import actionlib

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

    move_group.set_joint_value_target()

    [em,plan,pt,mm] = move_group.plan()
    print("plan ok, input")
    self.display_trajectory(plan)
    input()

    move_group.execute(plan, wait=True)

    move_group.stop()
    move_group.clear_pose_targets()

  def go_to_joint_pos(self,joint_goal):
    # self.move_group.go(joint_goal, wait=True)
    self.move_group.set_joint_value_target(joint_goal)
    [em, plan, pt, mm] = self.move_group.plan()

    print(plan)

    print("plan ok, input")
    input()

    client = actionlib.SimpleActionClient('ur5_moveit_lab/follow_joint_trajectory', control_msgs.msg.FollowJointTrajectoryAction)
    print('waitin for ur5_moveit_lab/follow_joint_trajectory')
    client.wait_for_server()

    goal = control_msgs.msg.FollowJointTrajectoryGoal

    goal.trajectory = plan

    print(goal)

    client.send_goal(goal)

    print('waitin')

    client.wait_for_result()


    # self.move_group.execute(plan, wait=True)
    # self.move_group.stop()
    # self.move_group.clear_pose_targets()


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

    jg = ur5interface.move_group.get_current_joint_values()

    print(jg)

    joint_goal = [-0.903442684804098, -1.4926427046405237, 1.5631790161132812, -1.6353634039508265, -1.545349423085348, 3.6319174766540527]

    ur5interface.go_to_joint_pos(joint_goal)

    print('motion ok. press enter to go home - carefully')
    input()

    pub = rospy.Publisher('/speed_ovr', Int64, queue_size=10)
    ovr = 25
    pub.publish(ovr)

    jg = [-2.200888458882467, -1.077665154133932, 1.4601397514343262, -1.9271062056170862, -1.5696128050433558, 2.334613561630249]

    ur5interface.go_to_joint_pos(jg)


    # current_pose = ur5interface.move_group.get_current_pose(ur5interface.move_group.get_end_effector_link())
    # print(current_pose)
    #
    # pose_goal = copy.deepcopy(current_pose)
    #
    # pose_goal.pose.position.x = pose_goal.pose.position.x + 0.05
    # pose_goal.pose.position.y = pose_goal.pose.position.y + 0.0
    # pose_goal.pose.position.z = pose_goal.pose.position.z - 0.0
    #
    # ur5interface.go_to_pose_goal(pose_goal)


  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return


if __name__ == "__main__":
  main()