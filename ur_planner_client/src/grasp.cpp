#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <vector>
#include <cmath>
#include <memory>
#include <string>

// plain struct holds all state (C-style)
struct GraspState {
  // ROS2 forces shared_ptr handles here
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub;
  moveit::planning_interface::PlanningSceneInterface scene;

  // add required fields
  // pre-grasp height offset
  // busy flag (bool) to avoid callnack overlap
  // last target pose to check if movement above threshold
  double offset = 0.15;
  bool busy = false;
  geometry_msgs::msg::PoseStamped last_target;
  bool has_last_target = false;
  double threshold = 0.01;

  std::string base_frame = "base_link";
  std::string tip_link = "tool0";
  double velocity_scale  = 0.6;
  double planning_time   = 5.0; 

  std::string cube_id = "cube";
  double cube_size = 0.05;          //match with synthetic camera
  bool   pose_is_top_face = false;  
  double grasp_clearance = 0.02;    // pause above cube top
  double eef_step = 0.01;           // cartesian interpolation step
  double min_fraction = 0.95;       // minimum acceptable Cartesian path fraction
  std::vector<std::string> touch_links = {"tool0", "wrist_3_link", "flange"};
};

// build the pre-grasp pose
// input : detected cube pose (PoseStamped), offset
// output: PoseStamped with z raised by offset and tool pointing down
geometry_msgs::msg::PoseStamped make_pregrasp(
    const geometry_msgs::msg::PoseStamped& cube, double offset)
    {
        geometry_msgs::msg::PoseStamped target;
        target.header = cube.header;
        target.pose.position.x = cube.pose.position.x;
        target.pose.position.y = cube.pose.position.y;
        target.pose.position.z = cube.pose.position.z + offset;

        target.pose.orientation.x = 1.0;
        target.pose.orientation.y = 0.0;
        target.pose.orientation.z = 0.0;
        target.pose.orientation.w = 0.0;

        return target;

    }

// plan and execute to a pose
// set pose target, call plan(), check the error code
// only call execute() if planning succeeded
// log success/failure with RCLCPP_INFO / RCLCPP_ERROR
// return planning status
bool plan_and_execute(GraspState& s, const geometry_msgs::msg::PoseStamped& target)
{
    if (target.header.frame_id != s.base_frame) {
        RCLCPP_ERROR(s.node->get_logger(),
                    "target frame '%s' != planning frame '%s'",
                    target.header.frame_id.c_str(), s.base_frame.c_str());
        return false;
    }
    s.move_group->setPoseTarget(target, s.tip_link);
    s.move_group->setMaxVelocityScalingFactor(s.velocity_scale);
    s.move_group->setPlanningTime(s.planning_time);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode ec = s.move_group->plan(plan);

    if (ec != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(s.node->get_logger(), "plan() failed, error code %d", ec.val);
        return false;
    }
    
    RCLCPP_INFO(s.node->get_logger(), "plan() OK, %zu waypoints",
              plan.trajectory.joint_trajectory.points.size());

    ec = s.move_group->execute(plan);
    if (ec != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(s.node->get_logger(), "execute() failed, error code %d", ec.val);
        return false;
    }
    return true;
}

// detach and remove previous cube
void reset_cube(GraspState& s)
{
    s.move_group->detachObject(s.cube_id);
    s.scene.removeCollisionObjects({s.cube_id});
}

// 3D box at the cube centre
void add_cube(GraspState& s, const geometry_msgs::msg::PoseStamped& cube_centre)
{
    moveit_msgs::msg::CollisionObject obj;
    obj.id = s.cube_id;
    obj.header.frame_id = s.base_frame;

    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {s.cube_size, s.cube_size, s.cube_size};

    obj.primitives.push_back(box);
    obj.primitive_poses.push_back(cube_centre.pose);
    obj.operation = moveit_msgs::msg::CollisionObject::ADD;
    s.scene.applyCollisionObject(obj);
}

// straight-line move to a pose; returns false if the path is incomplete
bool move_cartesian(GraspState& s, const geometry_msgs::msg::Pose& goal)
{
    std::vector<geometry_msgs::msg::Pose> waypoints{goal};
    moveit_msgs::msg::RobotTrajectory traj;
    double fraction = s.move_group->computeCartesianPath(waypoints, s.eef_step, traj);

    if (fraction < s.min_fraction) {
        RCLCPP_ERROR(s.node->get_logger(),
                     "Cartesian path only %.0f%% achievable", fraction * 100.0);
        return false;
    }
    moveit::core::MoveItErrorCode ec = s.move_group->execute(traj);
    if (ec != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(s.node->get_logger(), "Cartesian execute() failed, code %d", ec.val);
        return false;
    }
    return true;
}

// virtual gripper: the cube becomes part of the robot
void grasp(GraspState& s)
{
    s.move_group->attachObject(s.cube_id, s.tip_link, s.touch_links);
    RCLCPP_INFO(s.node->get_logger(), "cube attached to %s", s.tip_link.c_str());
}

bool pick_cube(GraspState& s, const geometry_msgs::msg::PoseStamped& detected)
{
    // cube centre, with a valid orientation for the collision box
    geometry_msgs::msg::PoseStamped cube = detected;
    if (s.pose_is_top_face) {
        cube.pose.position.z -= s.cube_size / 2.0;
    }
    cube.pose.orientation.x = 0.0;
    cube.pose.orientation.y = 0.0;
    cube.pose.orientation.z = 0.0;
    cube.pose.orientation.w = 1.0;

    reset_cube(s);
    add_cube(s, cube);

    // pre-grasp with the normal planner
    geometry_msgs::msg::PoseStamped pre = make_pregrasp(cube, s.offset);
    if (!plan_and_execute(s, pre)) return false;

    // straight down to just above the top face
    geometry_msgs::msg::Pose grasp_pose = pre.pose;
    grasp_pose.position.z = cube.pose.position.z + s.cube_size / 2.0 + s.grasp_clearance;
    if (!move_cartesian(s, grasp_pose)) return false;

    // grasp and lift
    grasp(s);
    return move_cartesian(s, pre.pose);
}


// subscription callback logic
// ignore if busy
// apply trigger policy
// set busy, call make_pregrasp + plan_and_execute, clear busy
void on_object_pose(GraspState& s, const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    // ignore the message if a plan/execute is already in progress
    if (s.busy) {
        return;
    }

    // trigger policy: skip if the cube has not moved enough since the last
    // successful grasp attempt.
    if (s.has_last_target) {
        double dx = msg->pose.position.x - s.last_target.pose.position.x;
        double dy = msg->pose.position.y - s.last_target.pose.position.y;
        double dz = msg->pose.position.z - s.last_target.pose.position.z;
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

        if (dist < s.threshold) {
        return;
        }
    }

    // build the pre-grasp pose
    geometry_msgs::msg::PoseStamped pre_grasp = make_pregrasp(*msg, s.offset);

    // plan and execute
    s.busy = true;
    bool done = pick_cube(s, *msg);
    s.busy = false;

    // update state only on success, a failed plan is retried on the next message
    if (done) {
        s.last_target = *msg;
        s.has_last_target = true;
        RCLCPP_INFO(s.node->get_logger(),
                    "moved to pre-grasp above cube at (%.3f, %.3f, %.3f)",
                    msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    } 
    else {
        RCLCPP_WARN(s.node->get_logger(), "pre-grasp attempt failed, will retry on next pose");
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    GraspState s;

    // ROS2 forces a class here (rclcpp::Node), held by shared_ptr.
    // automatically_declare_parameters_from_overrides lets the node accept the
    // robot_description / kinematics parameters that MoveIt needs.
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    s.node = std::make_shared<rclcpp::Node>("grasp_node", options);

    // create the MoveGroupInterface for the UR planning group
    s.move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        s.node, "ur_manipulator");

    s.base_frame = "base_link";
    s.tip_link   = "tool0";

    s.node->get_parameter_or("cube_size", s.cube_size, s.cube_size);
    s.node->get_parameter_or("pose_is_top_face", s.pose_is_top_face, s.pose_is_top_face);
    s.node->get_parameter_or("pre_grasp_height", s.offset, s.offset);
    s.node->get_parameter_or("grasp_clearance", s.grasp_clearance, s.grasp_clearance);
    s.node->get_parameter_or("touch_links", s.touch_links, s.touch_links);

    
    auto cb_group = s.node->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = cb_group;

    
    s.sub = s.node->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/object_pose", rclcpp::QoS(1),
        [&s](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            on_object_pose(s, msg);
        },
        sub_options);

    RCLCPP_INFO(s.node->get_logger(), "grasp_node ready, waiting for /object_pose");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(s.node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}