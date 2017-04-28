// Copyright 2017 Geoffrey Biggs (geoffrey.biggs@aist.go.jp)

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <string>
#include <vector>


int main(int argc, char **argv) {
  ros::init(argc, argv, "rsj_pick_and_place");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface arm("arm");
  moveit::planning_interface::PlanningSceneInterface scene;

  arm.setGoalTolerance(0.05);

  // Clear the planning scene
  std::vector<std::string> objs;
  for (auto o: scene.getObjects()) {
    objs.push_back(o.first);
  }
  for (auto o: scene.getAttachedObjects()) {
    objs.push_back(o.first);
  }
  scene.removeCollisionObjects(objs);

  moveit_msgs::CollisionObject sponge;
  sponge.header.frame_id = arm.getPlanningFrame();
  sponge.id = "sponge";
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.04;
  primitive.dimensions[1] = 0.04;
  primitive.dimensions[2] = 0.031;
  geometry_msgs::Pose pose;
  pose.position.x = 0.17;
  pose.position.y = 0.0;
  pose.position.z = 0.015;
  pose.orientation.w = 1;
  sponge.primitives.push_back(primitive);
  sponge.primitive_poses.push_back(pose);
  sponge.operation = sponge.ADD;
  scene.applyCollisionObject(sponge);

  moveit_msgs::CollisionObject table;
  table.header.frame_id = arm.getPlanningFrame();
  table.id = "table";
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 1;
  primitive.dimensions[1] = 1;
  primitive.dimensions[2] = 0.1;
  pose.position.x = 0;
  pose.position.y = 0;
  pose.position.z = -0.05;
  pose.orientation.w = 1;
  table.primitives.push_back(primitive);
  table.primitive_poses.push_back(pose);
  table.operation = table.ADD;
  std_msgs::ColorRGBA colour;
  colour.b = 0.5;
  scene.applyCollisionObject(table, colour);

  ROS_INFO("Added sponge and table to planning scene");
  ros::Duration(1).sleep();

  std::vector<moveit_msgs::Grasp> grasps;
  moveit_msgs::Grasp g;
  g.grasp_pose.header.frame_id = "base_link";
  g.grasp_pose.pose.position.x = 0.15;
  g.grasp_pose.pose.position.z = 0.05;
  g.grasp_pose.pose.orientation.y = 0.707106;
  g.grasp_pose.pose.orientation.w = 0.707106;

  g.pre_grasp_approach.direction.header.frame_id = "base_link";
  g.pre_grasp_approach.direction.vector.z = -1;
  g.pre_grasp_approach.min_distance = 0.05;
  g.pre_grasp_approach.desired_distance = 0.07;

  g.post_grasp_retreat.direction.header.frame_id = "base_link";
  g.post_grasp_retreat.direction.vector.z = 1;
  g.post_grasp_retreat.min_distance = 0.05;
  g.post_grasp_retreat.desired_distance = 0.07;

  g.pre_grasp_posture.joint_names.resize(1, "crane_plus_moving_finger_joint");
  g.pre_grasp_posture.points.resize(1);
  g.pre_grasp_posture.points[0].positions.resize(1);
  g.pre_grasp_posture.points[0].positions[0] = 0.1;

  g.grasp_posture.joint_names.resize(1, "crane_plus_moving_finger_joint");
  g.grasp_posture.points.resize(1);
  g.grasp_posture.points[0].positions.resize(1);
  g.grasp_posture.points[0].positions[0] = 0.01;

  grasps.push_back(g);
  arm.setSupportSurfaceName("table");
  ROS_INFO("Beginning pick");
  arm.pick("sponge", grasps);
  ROS_INFO("Pick done");

  std::vector<moveit_msgs::PlaceLocation> location;
  moveit_msgs::PlaceLocation p;
  p.place_pose.header.frame_id = arm.getPlanningFrame();
  p.place_pose.pose.position.x = 0.1;
  p.place_pose.pose.position.y = -0.2;
  p.place_pose.pose.position.z = 0.1;
  p.place_pose.pose.orientation.y = 0.707106;
  p.place_pose.pose.orientation.w = 0.707106;

  p.pre_place_approach.direction.header.frame_id = "base_link";
  p.pre_place_approach.direction.vector.z = -1;
  p.pre_place_approach.min_distance = 0.05;
  p.pre_place_approach.desired_distance = 0.07;

  p.post_place_retreat.direction.header.frame_id = "base_link";
  p.post_place_retreat.direction.vector.z = 1;
  p.post_place_retreat.min_distance = 0.05;
  p.post_place_retreat.desired_distance = 0.07;

  p.post_place_posture.joint_names.resize(1, "crane_plus_moving_finger_joint");
  p.post_place_posture.points.resize(1);
  p.post_place_posture.points[0].positions.resize(1);
  p.post_place_posture.points[0].positions[0] = 0.1;

  location.push_back(p);
  arm.setSupportSurfaceName("table");
  ROS_INFO("Beginning place");
  arm.place("sponge", location);
  ROS_INFO("Place done");

  ros::shutdown();
  return 0;
}
