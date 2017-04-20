#!/usr/bin/env python

"""
RSJ pick-and-place demonstration.

For the RSJ 2017 tutorial.

"""

import actionlib
import moveit_commander
import rospy
import sys
import tf2_geometry_msgs
import tf2_ros

from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from geometry_msgs.msg import Point, Pose2D, Pose, Quaternion, Transform, \
                              TransformStamped, Vector3Stamped
from tf.transformations import quaternion_from_euler

GRIPPER_OPEN_WIDTH = 0.1
GRIPPER_CLOSED_WIDTH = 0.0


def setup_arm(scene_frame):
    """Set up the arm for planning."""
    rospy.loginfo('[RSJPickAndPlace] Preparing arm for planning')
    # Set up the arm move group
    arm = moveit_commander.MoveGroupCommander('arm')
    # Allow flexibility in position and orientation to give MoveIt! more
    # freedom in planning
    arm.set_goal_position_tolerance(0.01)
    # arm.set_goal_orientation_tolerance(0.1)
    # arm.set_goal_joint_tolerance(0.05)
    # Allow replanning in case of failing to find a solution
    arm.allow_replanning(True)
    # Set the reference frame
    arm.set_pose_reference_frame(scene_frame)
    # Allow 5 seconds per planning attempt
    #arm.set_planning_time(5)

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


def make_finger_tip_offset(gripper_rotation, tf_buffer):
    """Calculate the finger tip offset from the gripper frame origin."""
    # Wait for tf data
    while not rospy.is_shutdown():
        rate = rospy.Rate(1)
        try:
            t = tf_buffer.lookup_transform(
                'crane_plus_fixed_finger_tip_link',
                'crane_plus_gripper_link',
                rospy.Time(0))
            break
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
    print('Transform is\n' + str(t))
    v = Vector3Stamped(vector=t.transform.translation)
    print('v is\n' + str(v))
    r = TransformStamped(transform=Transform(rotation=gripper_rotation))
    print('r is\n' + str(r))
    trans_vector = tf2_geometry_msgs.do_transform_vector3(v, r).vector
    print('transform is\n' + str(trans_vector))
    return Transform(translation=trans_vector, rotation=Quaternion(w=1))


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


def do_grasp(
        arm,
        gripper,
        object_pose,
        object_grasp_offset,
        grasp_prepare_z,
        grasp_z,
        gripper_rotation,
        tip_transform):
    """Grasp an object using the arm."""
    # Move to the prepare pose
    grasp_prepare_pose = Pose(Point(
            object_pose.x + tip_transform.translation.x,
            object_pose.y + tip_transform.translation.y - object_grasp_offset,
            grasp_prepare_z + tip_transform.translation.z),
        gripper_rotation)
    rospy.loginfo(
        '[RSJPickAndPlace] Moving arm to grasp-prepare pose:\n' +
        str(grasp_prepare_pose))
    arm.set_pose_target(grasp_prepare_pose)
    if not arm.go():
        rospy.logwarn('[RSJPickAndPlace] Failed to move to pose')
        return False
    # Move to the grasp pose
    grasp_pose = Pose(Point(
            object_pose.x + tip_transform.translation.x,
            object_pose.y + tip_transform.translation.y - object_grasp_offset,
            grasp_z + tip_transform.translation.z),
        gripper_rotation)
    rospy.loginfo(
        '[RSJPickAndPlace] Moving arm to grasp pose:\n' + str(grasp_pose))
    arm.set_pose_target(grasp_pose)
    if not arm.go():
        rospy.logwarn('[RSJPickAndPlace] Failed to move to pose')
        return False
    # Close the gripper
    close_gripper(gripper)
    # Move to the prepare position again
    rospy.loginfo(
        '[RSJPickAndPlace] Moving arm to grasp-prepare pose:\n' +
        str(grasp_prepare_pose))
    arm.set_pose_target(grasp_prepare_pose)
    if not arm.go():
        rospy.logwarn('[RSJPickAndPlace] Failed to move to pose')
        return False
    return True


def do_place(arm, gripper, place_pose):
    """Place an object using the arm."""
    # Move to the place position z+0.05
    place_prepare_pose = place_pose
    place_prepare_pose.position.z += 0.05
    rospy.loginfo(
        '[RSJPickAndPlace] Moving arm to place-prepare pose:\n' +
        str(place_prepare_pose))
    arm.set_pose_target(place_prepare_pose)
    if not arm.go():
        rospy.logwarn('[RSJPickAndPlace] Failed to move to pose')
        return False
    # Move to the place pose
    rospy.loginfo(
        '[RSJPickAndPlace] Moving arm to place pose:\n' + str(place_pose))
    arm.set_pose_target(place_pose)
    if not arm.go():
        rospy.logwarn('[RSJPickAndPlace] Failed to move to pose')
        return False
    # Open the gripper
    open_gripper(gripper)
    return True


def pick_and_place(object_pose, args):
    """Callback for the pose of a new object to pick-and-place."""
    arm, gripper, tf_buffer, place_pose, grasp_prepare_z, grasp_z, \
        gripper_rotation, tip_transform, max_reach, object_grasp_offset = args
    rospy.loginfo('[RSJPickAndPlace] Got new object pose:\n' + str(object_pose))
    if do_grasp(
            arm,
            gripper,
            object_pose,
            object_grasp_offset,
            grasp_prepare_z,
            grasp_z,
            gripper_rotation,
            tip_transform):
        do_place(arm, gripper, place_pose)
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
    grasp_prepare_z = rospy.get_param('~grasp_prepare_z', 0.07)
    grasp_z = rospy.get_param('~grasp_z', 0.02)
    max_reach = rospy.get_param('~max_reach', 0.23)
    object_grasp_offset = rospy.get_param('~object_grasp_offset', 0.03)
    gripper_rotation = Quaternion(*quaternion_from_euler(
        0,
        rospy.get_param('~gripper_angle', 1.8),
        0))

    # Listen to tf
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    # Make the gripper horizontal in the placing pose
    place_pose.orientation = gripper_rotation
    # Set up the arm
    arm = setup_arm(scene_frame)
    # Set up the gripper
    gripper, gripper_pose_pub = setup_gripper()
    # Calculate the offset from the fingertip to the gripper_link in the scene
    # frame
    tip_transform = make_finger_tip_offset(gripper_rotation, tf_buffer)

    # Subscribe to a topic providing poses of things to pick-and-place
    rospy.Subscriber(
        'objects',
        Pose2D,
        pick_and_place,
        (arm,
         gripper,
         tf_buffer,
         place_pose,
         grasp_prepare_z,
         grasp_z,
         gripper_rotation,
         tip_transform,
         max_reach,
         object_grasp_offset))
    rospy.loginfo('[RSJPickAndPlace] Ready to receive object poses')
    while not rospy.is_shutdown():
        rate.sleep()

    # Shut down MoveIt! cleanly
    moveit_commander.roscpp_shutdown()

    # All done
    return 0


if __name__ == '__main__':
    sys.exit(main())
