// src/planner_client.cpp
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// All program state lives in one plain struct.
struct PlannerState {
  // ROS2 FORCES A CLASS HERE: rclcpp::Node is a class, and MoveGroupInterface
  // needs a shared_ptr to one
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mgi;
  moveit::planning_interface::PlanningSceneInterface scene;

  std::string group      = "ur_manipulator";
  std::string base_frame = "base_link";
  std::string tip_link   = "tool0";
};

// Build a pose with the tool pointing straight down.
// tool0's z-axis points out of the flange. Rotating pi about x flips z to -z,
// so the flange faces the table. Quaternion is (x,y,z,w) = (1,0,0,0).
static geometry_msgs::msg::PoseStamped make_pose_down(
    const std::string& frame, double x, double y, double z, double yaw)
{
  tf2::Quaternion q;
  q.setRPY(M_PI, 0.0, yaw);  // yaw rotates the gripper about the vertical axis
  q.normalize();

  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = frame;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.position.z = z;
  p.pose.orientation = tf2::toMsg(q);
  return p;
}

// Add a box as a table. Top surface sits 1 cm BELOW base_link's origin;
// if it touched the robot base, the start state would be "in collision".
static void add_table(PlannerState& s)
{
  const double sx = 1.0, sy = 1.0, sz = 0.3;
  const double top_z = -0.01;

  moveit_msgs::msg::CollisionObject obj;
  obj.header.frame_id = s.base_frame;
  obj.id = "table";

  shape_msgs::msg::SolidPrimitive box;
  box.type = box.BOX;
  box.dimensions = {sx, sy, sz};

  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = top_z - sz / 2.0;

  obj.primitives.push_back(box);
  obj.primitive_poses.push_back(pose);
  obj.operation = obj.ADD;

  bool ok = s.scene.applyCollisionObject(obj);
  RCLCPP_INFO(s.node->get_logger(), "applyCollisionObject returned %d", (int)ok);
}

// Plan (and optionally execute) to a pose. Returns true on success.
static bool plan_to_pose(PlannerState& s,
                         const geometry_msgs::msg::PoseStamped& target,
                         bool do_execute)
{
  s.mgi->setPoseReferenceFrame(s.base_frame);
  s.mgi->setPoseTarget(target, s.tip_link);

  // plan() and execute() are separate calls and each returns its own error code.
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit::core::MoveItErrorCode ec = s.mgi->plan(plan);
  if (ec != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(s.node->get_logger(), "plan() failed, error code %d", ec.val);
    return false;
  }
  RCLCPP_INFO(s.node->get_logger(), "plan() OK, %zu waypoints",
              plan.trajectory.joint_trajectory.points.size());

  if (!do_execute) return true;

  ec = s.mgi->execute(plan);
  if (ec != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(s.node->get_logger(), "execute() failed, error code %d", ec.val);
    return false;
  }
  return true;
}

static double get_param(PlannerState& s, const std::string& name, double fallback)
{
  double v = fallback;
  s.node->get_parameter_or(name, v, fallback);  // uses fallback if not set
  return v;
}

// Run demo
static void run_demo(PlannerState& s)
{
  s.mgi->setPlanningTime(5.0);
  s.mgi->setNumPlanningAttempts(10);
  s.mgi->setMaxVelocityScalingFactor(0.6);
  s.mgi->setMaxAccelerationScalingFactor(0.6);

  double x = get_param(s, "target_x", 0.4);
  double y = get_param(s, "target_y", 0.2);
  double z = get_param(s, "target_z", 0.4);
  double yaw = get_param(s, "target_yaw", 0.0);
  auto target = make_pose_down(s.base_frame, x, y, z, yaw);

  double hx = get_param(s, "home_x", 0.4);
  double hy = get_param(s, "home_y", 0.2);
  double hz = get_param(s, "home_z", 0.4);
  double hyaw = get_param(s, "home_yaw", 0.0);
  auto home = make_pose_down(s.base_frame, hx, hy, hz, hyaw);

  RCLCPP_INFO(s.node->get_logger(), "--- 1: plan + execute, target position");
  plan_to_pose(s, target, true);

  RCLCPP_INFO(s.node->get_logger(), "--- 2: plan + execute, home position");
  plan_to_pose(s, home, true);

  RCLCPP_INFO(s.node->get_logger(), "--- 3: add table");
  add_table(s);
  std::this_thread::sleep_for(std::chrono::seconds(3));  // let the scene update

  RCLCPP_INFO(s.node->get_logger(), "--- 4: plan+execute, low position, table present");

  double a = get_param(s, "low_x", 0.4);
  double b = get_param(s, "low_y", 0.2);
  double c = get_param(s, "low_z", 0.4);
  double d = get_param(s, "low_yaw", 0.0);
  auto low = make_pose_down(s.base_frame, a, b, c, d);
  plan_to_pose(s, low, true);

  // Ends spin() in main so the process exits cleanly.
  rclcpp::shutdown();
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  PlannerState state;
  rclcpp::NodeOptions opts;
  opts.automatically_declare_parameters_from_overrides(true);
  state.node = std::make_shared<rclcpp::Node>("planner_client", opts);
  state.mgi  = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                   state.node, state.group);

  // plan() and execute() block while waiting for callbacks (joint states,
  // action results). Those callbacks only run while spin() is running, so the
  // blocking work goes on a worker thread and main thread spins.
  std::thread worker(run_demo, std::ref(state));
  rclcpp::spin(state.node);
  worker.join();
  return 0;
}