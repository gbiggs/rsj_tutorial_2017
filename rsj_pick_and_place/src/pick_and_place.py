#!/usr/bin/env python

"""
RSJ pick-and-place demonstration.

For the RSJ 2017 tutorial.

"""

import actionlib
import math
import moveit_commander
import rospy
import sys
import tf2_ros

from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from geometry_msgs.msg import Point, Pose2D, Pose, Quaternion
from moveit_msgs.msg import PlanningScene
from moveit_python import PlanningSceneInterface
from tf.transformations import quaternion_from_euler

GRIPPER_OPEN_WIDTH = 0.1
GRIPPER_CLOSED_WIDTH = 0.0


def setup_planning_scene(scene_frame):
    """Create a planning scene and add a table below the arm."""
    rospy.loginfo('[RSJPickAndPlace] Setting up planning scene')
    scene = PlanningSceneInterface(scene_frame)
    # Publish changes to the scene
    scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=1)
    # Place a table in the scene (so that the arm doesn't try to go through it)
    scene.addBox('table', 0, 0, 0, 1, 1, 0.01, wait=True)

    return scene, scene_pub


def setup_arm(scene_frame):
    """Set up the arm for planning."""
    rospy.loginfo('[RSJPickAndPlace] Preparing arm for planning')
    # Set up the arm move group
    arm = moveit_commander.MoveGroupCommander('arm')
    # Allow flexibility in position and orientation to give MoveIt! more
    # freedom in planning
    # arm.set_goal_position_tolerance(0.05)
    # arm.set_goal_orientation_tolerance(0.1)
    # arm.set_goal_joint_tolerance(0.05)
    # Allow replanning in case of failing to find a solution
    arm.allow_replanning(True)
    # Set the reference frame
    arm.set_pose_reference_frame(scene_frame)
    # Allow 5 seconds per planning attempt
    arm.set_planning_time(5)

    # Start the up vertically up
    arm.set_named_target('vertical')
    arm.go()

    return arm


def setup_gripper():
    """Set up the gripper ready for gripping."""
    rospy.loginfo('[RSJPickAndPlace] Preparing gripper for gripping')
    # Create a client for the gripper action
    gripper_client = actionlib.SimpleActionClient(
        '/gripper_command',
        GripperCommandAction)
    # Publish gripper poses
    gripper_pose_pub = rospy.Publisher(
        'gripper_pose',
        Pose,
        queue_size=1)

    # Set the gripper to open
    goal = GripperCommandGoal()
    goal.command.position = GRIPPER_OPEN_WIDTH
    gripper_client.send_goal(goal)
    gripper_client.wait_for_result(rospy.Duration(2))

    return gripper_client, gripper_pose_pub


def open_gripper(gripper_client):
    """Open the gripper."""
    rospy.loginfo('[RSJPickAndPlace] Opening gripper')
    goal = GripperCommandGoal()
    goal.command.position = GRIPPER_OPEN_WIDTH
    gripper_client.send_goal(goal)
    gripper_client.wait_for_result()


def close_gripper(gripper_client):
    """Close the gripper."""
    rospy.loginfo('[RSJPickAndPlace] Closing gripper')
    goal = GripperCommandGoal()
    goal.command.position = GRIPPER_CLOSED_WIDTH
    gripper_client.send_goal(goal)
    gripper_client.wait_for_result()


def gripper_grasp_orientation():
    """Provide an orientation for the gripper suitable for grasping."""
    return Quaternion(*quaternion_from_euler(0, math.pi/2, 0))


def do_grasp(arm, object_pose, grasp_prepare_z, grasp_z, gripper):
    """Move the arm to the grasping position for an object."""
    # Move to the prepare pose
    grasp_prepare_pose = Pose(Point(
        object_pose.x, object_pose.y, grasp_prepare_z),
        gripper_grasp_orientation())
    rospy.loginfo(
        '[RSJPickAndPlace] Moving arm to grasp-prepare pose: ' +
        str(grasp_prepare_pose))
    arm.set_pose_target(grasp_prepare_pose)
    if not arm.go():
        rospy.logwarn('[RSJPickAndPlace] Failed to move to pose')
        return False
    # Move to the grasp pose
    grasp_pose = Pose(Point(
        object_pose.x, object_pose.y, grasp_z),
        gripper_grasp_orientation())
    rospy.loginfo(
        '[RSJPickAndPlace] Moving arm to grasp pose: ' + str(grasp_pose))
    arm.set_pose_target(grasp_pose)
    if not arm.go():
        rospy.logwarn('[RSJPickAndPlace] Failed to move to pose')
        return False
    # Close the gripper
    close_gripper(gripper)
    # Move to the prepare position again
    rospy.loginfo(
        '[RSJPickAndPlace] Moving arm to grasp-prepare pose: ' +
        str(grasp_prepare_pose))
    arm.set_pose_target(grasp_prepare_pose)
    if not arm.go():
        rospy.logwarn('[RSJPickAndPlace] Failed to move to pose')
        return False
    return True


def do_place(arm, place_pose, gripper):
    """Move the arm to the pose for placing an object."""
    # Move to the place position z+0.05
    place_prepare_pose = place_pose
    place_prepare_pose.position.z += 0.05
    rospy.loginfo(
        '[RSJPickAndPlace] Moving arm to place-prepare pose: ' +
        str(place_prepare_pose))
    arm.set_pose_target(place_prepare_pose)
    if not arm.go():
        rospy.logwarn('[RSJPickAndPlace] Failed to move to pose')
        return False
    # Move to the place pose
    rospy.loginfo(
        '[RSJPickAndPlace] Moving arm to place pose: ' + str(place_pose))
    arm.set_pose_target(place_pose)
    if not arm.go():
        rospy.logwarn('[RSJPickAndPlace] Failed to move to pose')
        return False
    # Open the gripper
    open_gripper(gripper)
    return True


def pick_and_place(object_pose, args):
    """Callback for the pose of a new object to pick-and-place."""
    arm, gripper, scene, tf_buffer, place_pose, grasp_prepare_z, grasp_z, \
        max_reach = args
    rospy.loginfo('[RSJPickAndPlace] Got new object pose: ' + str(object_pose))
    if do_grasp(arm, object_pose, grasp_prepare_z, grasp_z, gripper):
        do_place(arm, place_pose, gripper)
    # Make sure the gripper is open even if moving failed
    open_gripper(gripper)
    # Move the arm to an intermediate position
    rospy.loginfo('[RSJPickAndPlace] Moving arm to resting pose')
    arm.set_named_target('resting')
    if not arm.go():
        rospy.logwarn('[RSJPickAndPlace] Failed to move to pose')
    rospy.loginfo(
        '[RSJPickAndPlace] Pick-and-place process finished, ready for '
        'next object')


def main():
    """Main entry point."""
    # Initialise MoveIt! and the node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('rsj_pick_and_place')

    # Get the task frames and other configuration from the parameter server
    scene_frame = rospy.get_param('~scene_task_frame', 'world')
    rate = rospy.Rate(rospy.get_param('~execution_rate', 1))
    place_pose = Pose()
    place_pose.position.x = rospy.get_param('~place_pose/x', 0.1)
    place_pose.position.y = rospy.get_param('~place_pose/y', -0.2)
    place_pose.position.z = rospy.get_param('~place_pose/z', 0.05)
    grasp_prepare_z = rospy.get_param('~grasp_prepare_z', 0.1)
    grasp_z = rospy.get_param('~grasp_z', 0.04)
    max_reach = rospy.get_param('~max_reach', 0.23)

    # Listen to tf
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    # Make the gripper horizontal in the placing pose
    place_pose.orientation = gripper_grasp_orientation()
    # Set up the arm
    arm = setup_arm(scene_frame)
    # Set up the gripper
    gripper, gripper_pose_pub = setup_gripper()
    # Create a planning scene for MoveIt! to plan in
    scene = setup_planning_scene(scene_frame)
    # Wait for the scene to catch up
    rospy.sleep(5)

    # Subscribe to a topic providing poses of things to pick-and-place
    rospy.Subscriber(
        'objects',
        Pose2D,
        pick_and_place,
        (arm,
         gripper,
         scene,
         tf_buffer,
         place_pose,
         grasp_prepare_z,
         grasp_z,
         max_reach))
    rospy.loginfo('[RSJPickAndPlace] Ready to receive object poses')
    while not rospy.is_shutdown():
        rate.sleep()

    # Shut down MoveIt! cleanly
    moveit_commander.roscpp_shutdown()

    # All done
    return 0


if __name__ == '__main__':
    sys.exit(main())
