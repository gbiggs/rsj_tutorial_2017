// Copyright 2017 Geoffrey Biggs (geoffrey.biggs@aist.go.jp)

#include <ros/ros.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <shape_msgs/SolidPrimitive.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <string>
#include <vector>


class RSJPickNPlacer {
 public:
  typedef actionlib::SimpleActionClient<control_msgs::GripperCommandAction> GripperClient;

  RSJPickNPlacer()
      : arm_("arm"),
        gripper_("/gripper_command", "true"),
        gripper_closed_width_(0.01),
        gripper_open_width_(0.1),
        prepare_z_(0.1),
        grasp_z_(0.04) {
    scene_frame_ = "base_link";
    place_position_.x = 0.1;
    place_position_.y = -0.2;
    place_position_.z = 0.05;
    SetupPlanningScene();
    SetupArm();
    SetupGripper();
  }

  void DoPickAndPlace(geometry_msgs::Vector3 const& location) {
    ROS_INFO("Received an object to pick-and-place");
    AddBoxToScene(location);
    if (GraspObject(location)) {
      PlaceObject();
    }
    arm_.setNamedTarget("resting");
    arm_.move();
    RemoveBoxFromScene();
    ROS_INFO("Pick-and-place process finished, ready for next object");
  }

 private:
  std::string scene_frame_;
  moveit::planning_interface::PlanningSceneInterface scene_;
  moveit::planning_interface::MoveGroupInterface arm_;

  GripperClient gripper_;
  float gripper_closed_width_;
  float gripper_open_width_;

  geometry_msgs::Vector3 place_position_;
  float prepare_z_;
  float grasp_z_;

  bool DoGripperAction(float width) {
    control_msgs::GripperCommandGoal goal;
    goal.command.position = width;
    gripper_.sendGoal(goal);
    bool finishedBeforeTimeout = gripper_.waitForResult(ros::Duration(30));
    if (!finishedBeforeTimeout) {
      ROS_INFO("Gripper action did not complete");
      return false;
    }
    return true;
  }

  geometry_msgs::Quaternion GripperOrientation() {
    geometry_msgs::Quaternion result;
    result.x = 0;
    result.y = 0.707106781187;
    result.z = 0;
    result.w = 0.707106781187;
    return result;
  }

  void SetupPlanningScene() {
    ROS_INFO("Setting up planning scene");
    // Clear the planning scene
    std::vector<std::string> objs;
    for (auto o: scene_.getObjects()) {
      objs.push_back(o.first);
    }
    for (auto o: scene_.getAttachedObjects()) {
      objs.push_back(o.first);
    }
    scene_.removeCollisionObjects(objs);

    moveit_msgs::CollisionObject table;
    table.header.frame_id = scene_frame_;
    table.id = "table";
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 1;
    primitive.dimensions[1] = 1;
    primitive.dimensions[2] = 0.1;
    geometry_msgs::Pose pose;
    pose.position.x = 0;
    pose.position.y = 0;
    pose.position.z = -0.05;
    pose.orientation.w = 1;
    table.primitives.push_back(primitive);
    table.primitive_poses.push_back(pose);
    table.operation = table.ADD;
    std_msgs::ColorRGBA colour;
    colour.b = 0.5;
    colour.a = 1;
    scene_.applyCollisionObject(table, colour);
  }

  void SetupArm() {
    ROS_INFO("Setting up arm for planning");
    arm_.setPoseReferenceFrame(scene_frame_);

    arm_.setNamedTarget("vertical");
    arm_.move();
  }

  void SetupGripper() {
    ROS_INFO("Setting up gripper");
    gripper_.waitForServer();
  }

  void AddBoxToScene(geometry_msgs::Vector3 const& location) {
    moveit_msgs::CollisionObject sponge;
    sponge.header.frame_id = scene_frame_;
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
    scene_.applyCollisionObject(sponge);
    ros::Duration(1).sleep();
  }

  void RemoveBoxFromScene() {
    std::vector<std::string> objs;
    objs.push_back("sponge");
    scene_.removeCollisionObjects(objs);
  }

  bool OpenGripper() {
    ROS_INFO("Opening gripper");
    return DoGripperAction(gripper_open_width_);
  }

  bool CloseGripper() {
    ROS_INFO("Closing gripper");
    return DoGripperAction(gripper_closed_width_);
  }

  bool GraspObject(geometry_msgs::Vector3 const& object_position) {
    // Open the gripper
    if (!OpenGripper()) {
      ROS_INFO("Failed to open gripper");
      return false;
    }
    // Move to the approach pose
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = scene_frame_;
    pose.pose.position.x = object_position.x - 0.02;
    pose.pose.position.y = object_position.y;
    pose.pose.position.z = prepare_z_;
    pose.pose.orientation = GripperOrientation();
    ROS_INFO("Moving arm to grasp-approach pose");
    arm_.setPoseTarget(pose);
    if (!arm_.move()) {
      ROS_INFO("Failed to move to grasp-approach pose");
      return false;
    }
    // Move to the grasp pose
    pose.pose.position.z = grasp_z_;
    ROS_INFO("Moving arm to grasp pose");
    arm_.setPoseTarget(pose);
    if (!arm_.move()) {
      ROS_INFO("Failed to move to grasp pose");
      return false;
    }
    // Close the gripper
    if (!CloseGripper()) {
      ROS_INFO("Failed to close gripper");
      return false;
    }
    // Move to the retreat pose
    pose.pose.position.z = prepare_z_;
    ROS_INFO("Moving arm to grasp-retreat pose");
    arm_.setPoseTarget(pose);
    if (!arm_.move()) {
      ROS_INFO("Failed to move to grasp-retreat pose");
      return false;
    }
    return true;
  }

  bool PlaceObject() {
    // Move to the approach pose
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = scene_frame_;
    pose.pose.position.x = place_position_.x;
    pose.pose.position.y = place_position_.y;
    pose.pose.position.z = prepare_z_;
    pose.pose.orientation = GripperOrientation();
    ROS_INFO("Moving arm to place-approach pose");
    arm_.setPoseTarget(pose);
    if (!arm_.move()) {
      ROS_INFO("Failed to move to place-approach pose");
      return false;
    }
    // Move to the place pose
    pose.pose.position.z = place_position_.z;
    ROS_INFO("Moving arm to place pose");
    arm_.setPoseTarget(pose);
    if (!arm_.move()) {
      ROS_INFO("Failed to move to place pose");
      return false;
    }
    // Open the gripper
    if (!OpenGripper()) {
      ROS_INFO("Failed to open gripper");
      return false;
    }
    // Move to the retreat pose
    pose.pose.position.z = prepare_z_;
    ROS_INFO("Moving arm to place-retreat pose");
    arm_.setPoseTarget(pose);
    if (!arm_.move()) {
      ROS_INFO("Failed to move to place-retreat pose");
      return false;
    }
    return true;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "rsj_pick_and_place");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  RSJPickNPlacer pnp;
  geometry_msgs::Vector3 obj;
  obj.x = 0.2;
  obj.y = 0;
  obj.z = 0;
  pnp.DoPickAndPlace(obj);

  ros::shutdown();
  return 0;
}
