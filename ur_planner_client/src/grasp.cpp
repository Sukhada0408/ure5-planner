#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <cmath>
#include <memory>
#include <string>

// Plain struct holds all state (C-style)
struct GraspState {
  // ROS2 forces shared_ptr handles here
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub;

  // TODO 1: add fields you need:
  //   - pre-grasp height offset (double, 0.15 m)
  //   - busy flag (bool) so callbacks don't overlap
  //   - last target pose (for the "moved enough?" check)
  double offset = 0.15;
  bool busy = false;
  geometry_msgs::msg::PoseStamped last_target;
  bool has_last_target = false;
  double threshold = 0.01;

  std::string base_frame = "base_link";
  std::string tip_link = "tool0";
  double velocity_scale  = 0.6;
  double planning_time   = 5.0; 
};

// TODO 2: free function: build the pre-grasp pose
//   input : detected cube pose (PoseStamped), offset
//   output: PoseStamped with z raised by offset and tool pointing down
//   Reuse your Stage 2 quaternion. Keep the frame_id.
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

// TODO 3: free function: plan and execute to a pose
//   - set pose target, call plan(), CHECK the error code
//   - only call execute() if planning succeeded
//   - log success/failure with RCLCPP_INFO / RCLCPP_ERROR
//   - return bool
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

// TODO 4: free function: the subscription callback logic
//   - ignore if busy
//   - apply your trigger policy from Step 1
//   - set busy, call make_pregrasp + plan_and_execute, clear busy
void on_object_pose(GraspState& s, const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    // 1. Ignore the message if a plan/execute is already in progress
    if (s.busy) {
        return;
    }

    // 2. Trigger policy: skip if the cube has not moved enough since the last
    //    successful grasp attempt. This check only applies if there IS a last target.
    if (s.has_last_target) {
        double dx = msg->pose.position.x - s.last_target.pose.position.x;
        double dy = msg->pose.position.y - s.last_target.pose.position.y;
        double dz = msg->pose.position.z - s.last_target.pose.position.z;
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

        if (dist < s.threshold) {
        return;
        }
    }

    // 3. Build the pre-grasp pose (above the cube, tool pointing down)
    geometry_msgs::msg::PoseStamped pre_grasp = make_pregrasp(*msg, s.offset);

    // 4. Plan and execute, guarded by the busy flag
    s.busy = true;
    bool done = plan_and_execute(s, pre_grasp);
    s.busy = false;

    // 5. Update state only on success, so a failed plan is retried on the next message
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

    // TODO 5a: create the node. ROS2 forces a class here (rclcpp::Node), held by shared_ptr.
    //          automatically_declare_parameters_from_overrides lets the node accept the
    //          robot_description / kinematics parameters that MoveIt needs.
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    s.node = std::make_shared<rclcpp::Node>("grasp_node", options);

    // TODO 5b: create the MoveGroupInterface for the UR planning group
    s.move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        s.node, "ur_manipulator");

    // Optional: override tunables here (defaults already live in the struct)
    s.base_frame = "base_link";
    s.tip_link   = "tool0";

    // TODO 5c: a dedicated callback group for the subscription, so the blocking
    //          plan()/execute() inside the callback does not occupy the same "slot"
    //          that other callbacks of this node need.
    auto cb_group = s.node->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = cb_group;

    // TODO 5d: the subscription. Queue depth 1 means that while the robot is moving,
    //          only the newest pose is kept and stale ones are dropped.
    //          The lambda forwards to the free function, capturing the state by reference
    //          (s outlives the executor, so this is safe).
    s.sub = s.node->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/object_pose", rclcpp::QoS(1),
        [&s](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            on_object_pose(s, msg);
        },
        sub_options);

    RCLCPP_INFO(s.node->get_logger(), "grasp_node ready, waiting for /object_pose");

    // TODO 6: spin with several threads, so that the node can still process
    //         other callbacks (e.g. state updates) while one thread is blocked
    //         inside plan()/execute().
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(s.node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}