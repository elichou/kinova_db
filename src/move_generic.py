#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from random import *

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True




class MoveGroupPythonInterface(object):

    def __init__(self):
        super(MoveGroupPythonInterface, self).__init__()

        # initialize moveit_commander and rospy node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface', anonymous=True)

        # instantiate a RobotCommander object
        # infos : kinematic model, current joint state
        robot = moveit_commander.RobotCommander()

        # instantiate a PlanningSceneInterface object
        # infos : getting, setting, updating internal understanding of robot
        scene = moveit_commander.PlanningSceneInterface()

        # instantiate MoveGroupCommander object
        # infos : interface to a planning group
        group_name = "arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        # create DisplayTrajectory ros publisher
        # infos :  display trajectories in Rviz
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                        moveit_msgs.msg.DisplayTrajectory,
                                                        queue_size = 20)

        # getting basic information
        planning_frame = move_group.get_planning_frame()
        print("====== Planning frame: %s", planning_frame)

        eef_link = move_group.get_end_effector_link()
        print("===== End effector link: %s", eef_link)

        group_names = robot.get_group_names()
        print("===== Available planning groups:", group_names)

        print("===== Printing robot state")
        print(robot.get_current_state())
        print("")


        # variables
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_pose_goal(self, pose):
        # planning to a pose goal
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = pose[0]
        pose_goal.position.x = pose[1]
        pose_goal.position.y = pose[2]
        pose_goal.position.z = pose[3]

        self.move_group.set_pose_target(pose_goal)

        # computation of the plan
        plan = self.move_group.go(wait=True)
        # avoid residual movement
        self.move_group.stop()
        # clear targets
        self.move_group.clear_pose_targets()

        # for testing
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def plan_cartesian_path(self, waypoints):
        # planning cartesian path
        (plan, fraction) = self.move_group.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0)         # jump_threshold
        return plan, fraction

    def display_trajectory(self, plan):
        # display a trajectory
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # publish
        self.display_trajectory_publisher.publish(display_trajectory)

    def execute_plan(self, plan):
        # executing a plan
        self.move_group.execute(plan, wait=True)

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        # ensuring collision updates are received
        start = rospy.get_time()
        seconds = rospy.get_time()

        while (seconds - start < timeout) and not rospy.is_shutdown():
            # test if box is in atteched objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # test if box is in the scene
            is_known = box_name in scene.get_known_object_names()

            # test if state is as expected
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return

            # sleep
            rospy.sleep(0.1)
            seconds = rospy.rospy.get_time()

        # in cas of timeout
        return False

    def star_waypoints(self, scale = 1):
        # array of star movement around random position
        waypoints = []

        # save current pose
        wpose = self.move_group.get_current_pose().pose
        waypoints.append(copy.deepcopy(wpose))

        for i in range(5):
            p = geometry_msgs.msg.Pose()
            p.orientation.w = 1.0
            p.position.x = scale*random()
            p.position.y = scale*random()
            p.position.z = scale*random()
            waypoints.append(copy.deepcopy(p))
            for j in range(10):
                pp = geometry_msgs.msg.Pose()
                pp.orientation.w = 1.0
                pp.position.x = 0.1*random() - p.position.x
                pp.position.y = 0.1*random() - p.position.y
                pp.position.z = 0.1*random() - p.position.z
                waypoints.append(copy.deepcopy(pp))
                waypoints.append(copy.deepcopy(p))

        waypoints.append(copy.deepcopy(wpose))

        return waypoints

    def line_waypoints(self):
        # line on xy yz and xz plan
        waypoints = []
        wpose = self.move_group.get_current_pose().pose
        waypoints.append(copy.deepcopy(wpose))

        p = geometry_msgs.msg.Pose()
        p.orientation.w = 1.0
        p.position.x = wpose.position.x - 2
        p.position.y = wpose.position.y
        p.position.z = wpose.position.z
        waypoints.append(copy.deepcopy(p))

        return waypoints
