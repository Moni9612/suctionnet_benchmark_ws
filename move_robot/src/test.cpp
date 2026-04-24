#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp/rclcpp.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <thread>
#include <atomic>

#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Geometry>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

// Function to interpolate between two quaternions
geometry_msgs::msg::Quaternion interpolateQuaternion(
    const geometry_msgs::msg::Quaternion& start,
    const geometry_msgs::msg::Quaternion& end, double t) {
    tf2::Quaternion start_q(start.x, start.y, start.z, start.w);
    tf2::Quaternion end_q(end.x, end.y, end.z, end.w);

    // Normalize the quaternions to avoid errors in interpolation
    start_q.normalize();
    end_q.normalize();

    // Perform spherical linear interpolation
    tf2::Quaternion interpolated_q = start_q.slerp(end_q, t);

    // Convert to geometry_msgs format
    geometry_msgs::msg::Quaternion result;
    result.x = interpolated_q.x();
    result.y = interpolated_q.y();
    result.z = interpolated_q.z();
    result.w = interpolated_q.w();

    return result;
}

int main(int argc, char** argv)
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_cat_robot", node_options);

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("move_cat_robot");

  // Create the MoveIt MoveGroup Interface
  static const std::string PLANNING_GROUP = "ur_manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
  
  // Use a shared pointer for the executor
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(move_group_node);

  // Use a thread to spin the executor
  std::thread spin_thread([&executor]() { executor->spin(); });

  // We can print the name of the reference frame for this robot.
  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());
  
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // We can get a list of all the groups in the robot:
  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  // Create collision object for the robot to avoid
  auto const collision_object1 = [frame_id =
                                 move_group.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject collision_object1;
    collision_object1.header.frame_id = frame_id;
    collision_object1.id = "box1";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 2.0;
    primitive.dimensions[primitive.BOX_Y] = 2.0;
    primitive.dimensions[primitive.BOX_Z] = 0.5;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose1;
    box_pose1.orientation.w = 1.0;  // We can leave out the x, y, and z components of the quaternion since they are initialized to 0
    box_pose1.position.x = 0;
    box_pose1.position.y = 0;
    box_pose1.position.z = -0.45;

    collision_object1.primitives.push_back(primitive);
    collision_object1.primitive_poses.push_back(box_pose1);
    collision_object1.operation = collision_object1.ADD;

    return collision_object1;
  }();
  
  planning_scene_interface.applyCollisionObject(collision_object1);

  // Create collision object for the robot to avoid
  auto const collision_object2 = [frame_id =
                                 move_group.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject collision_object2;
    collision_object2.header.frame_id = frame_id;
    collision_object2.id = "box2";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 2.0;
    primitive.dimensions[primitive.BOX_Y] = 0.05;
    primitive.dimensions[primitive.BOX_Z] = 2.0;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose2;
    box_pose2.orientation.w = 1.0;  // We can leave out the x, y, and z components of the quaternion since they are initialized to 0
    box_pose2.position.x = 0;
    box_pose2.position.y = -0.85;
    box_pose2.position.z = 0;

    collision_object2.primitives.push_back(primitive);
    collision_object2.primitive_poses.push_back(box_pose2);
    collision_object2.operation = collision_object2.ADD;

    return collision_object2;
  }();  
    
  planning_scene_interface.applyCollisionObject(collision_object2);
    
  //Create collision object for the robot to avoid
  auto const attached_object = [frame_id =
                                 move_group.getPlanningFrame()] {
    // Create attached collision object
    moveit_msgs::msg::AttachedCollisionObject attached_object;
    attached_object.link_name = "wrist_3_link";  // Attach to wrist_3_link
    attached_object.object.header.frame_id = "wrist_3_link";
    attached_object.object.id = "box3";
    shape_msgs::msg::SolidPrimitive primitive;

    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.08;  // Size in meters
    primitive.dimensions[primitive.BOX_Y] = 0.08;
    primitive.dimensions[primitive.BOX_Z] = 0.14;

    // Define the pose of the collision object relative to wrist_3_link
    geometry_msgs::msg::Pose box_pose3;
    box_pose3.orientation.w = 1.0;  // No rotation (identity quaternion)
    box_pose3.position.x = 0.0;     // Adjust as needed
    box_pose3.position.y = 0.0;     // Adjust as needed
    box_pose3.position.z = 0.07;    // Center of the box along its height

    // Add the shape and pose to the collision object
    attached_object.object.primitives.push_back(primitive);
    attached_object.object.primitive_poses.push_back(box_pose3);
    attached_object.object.operation = attached_object.object.ADD;

    return attached_object;
  }();  
   
  // Add the collision object to the scene
  planning_scene_interface.applyAttachedCollisionObject(attached_object);

  // Create collision object for the robot to avoid
  auto const attached_object1 = [frame_id =
                                 move_group.getPlanningFrame()] {
    // Create attached collision object
    moveit_msgs::msg::AttachedCollisionObject attached_object1;
    attached_object1.link_name = "wrist_3_link";  // Attach to wrist_3_link
    attached_object1.object.header.frame_id = "wrist_3_link";
    attached_object1.object.id = "box4";
    shape_msgs::msg::SolidPrimitive primitive;

    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.12;  // Size in meters
    primitive.dimensions[primitive.BOX_Y] = 0.09;
    primitive.dimensions[primitive.BOX_Z] = 0.07;

    // Define the pose of the collision object relative to wrist_3_link
    geometry_msgs::msg::Pose box_pose4;
    box_pose4.orientation.w = 1.0;  // No rotation (identity quaternion)
    box_pose4.position.x = -0.06;     // Adjust as needed
    box_pose4.position.y = 0.0;     // Adjust as needed
    box_pose4.position.z = 0.045;    // Center of the box along its height

    // Add the shape and pose to the collision object
    attached_object1.object.primitives.push_back(primitive);
    attached_object1.object.primitive_poses.push_back(box_pose4);
    attached_object1.object.operation = attached_object1.object.ADD;

    return attached_object1;
  }();  

  // Add the collision object to the scene
  planning_scene_interface.applyAttachedCollisionObject(attached_object1);
  
  // Clear previous states
  //move_group.clearPoseTargets();
  
  // Log the current state
  RCLCPP_INFO(LOGGER, "Current Pose of End-Effector:");
  geometry_msgs::msg::Pose current_pose = move_group.getCurrentPose().pose;
  
  RCLCPP_INFO(LOGGER, "Position: [%.4f, %.4f, %.4f]", 
              current_pose.position.x, current_pose.position.y, current_pose.position.z);
  RCLCPP_INFO(LOGGER, "Orientation: [%.4f, %.4f, %.4f, %.4f]", 
              current_pose.orientation.x, current_pose.orientation.y, 
              current_pose.orientation.z, current_pose.orientation.w);
  
  //Create a plan to that target pose

  geometry_msgs::msg::Pose target_pose;
 
  target_pose.position.x = -0.2878;
  target_pose.position.y = 0.8292;
  target_pose.position.z = 0.4831;
  target_pose.orientation.x = -0.6942;
  target_pose.orientation.y = 0.7197;
  target_pose.orientation.z = 0.0016;     
  target_pose.orientation.w = -0.00105; 
  
  /*target_pose.orientation.x = current_pose.orientation.x;
  target_pose.orientation.y = current_pose.orientation.y;
  target_pose.orientation.z = current_pose.orientation.z;     
  target_pose.orientation.w = current_pose.orientation.w;

  target_pose.orientation.x = 1;
  target_pose.orientation.y = 0;
  target_pose.orientation.z = 0;     
  target_pose.orientation.w = 0;*/
  
  move_group.setPoseTarget(target_pose);

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(current_pose);
  
  // Add intermediate waypoints with interpolated orientations
  for (int i = 1; i <= 10; ++i) {
    double t = static_cast<double>(i) / 11.0;
    geometry_msgs::msg::Pose intermediate_pose;
    intermediate_pose.position.x = current_pose.position.x + i * (target_pose.position.x - current_pose.position.x) / 11;
    intermediate_pose.position.y = current_pose.position.y + i * (target_pose.position.y - current_pose.position.y) / 11;
    intermediate_pose.position.z = current_pose.position.z + i * (target_pose.position.z - current_pose.position.z) / 11;

    // Interpolate the orientation
    intermediate_pose.orientation = interpolateQuaternion(current_pose.orientation, target_pose.orientation, t);
    waypoints.push_back(intermediate_pose); // Add the intermediate waypoint
  }
  
  // Add the final pose
  waypoints.push_back(target_pose);

  rclcpp::sleep_for(std::chrono::milliseconds(500));

  const double eef_step = 0.02;             // Step size for end-effector
  const double jump_threshold =1.14159;         // Threshold to ignore jumps in IK solutions
  move_group.setPlanningTime(100.0);  // Increase planning time (in seconds)
  const int max_planning_attempts = 10;       // Maximum number of planning attempts (increased)
  
  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = 0.0;
  bool plan_success = false;

  // Try multiple planning attempts
  for (int attempt = 0; attempt < max_planning_attempts; ++attempt) {
    fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    RCLCPP_INFO(LOGGER, "Plan (Cartesian path) (%.2f%% achieved) on attempt %d", fraction * 100.0, attempt + 1);
    
    for (size_t i = 0; i < waypoints.size(); ++i) {
    RCLCPP_INFO(LOGGER, "Waypoint %ld: Position [%.4f, %.4f, %.4f], Orientation [%.4f, %.4f, %.4f, %.4f]",
                i, waypoints[i].position.x, waypoints[i].position.y, waypoints[i].position.z,
                waypoints[i].orientation.x, waypoints[i].orientation.y, waypoints[i].orientation.z, waypoints[i].orientation.w);
    }


    if (fraction >= 0.90) {
      plan_success = true;
      break;  // Exit the loop if the plan is successful
    }

    RCLCPP_WARN(LOGGER, "Planning attempt %d failed. Trying again...", attempt + 1);
    rclcpp::sleep_for(std::chrono::milliseconds(500));  // Sleep before retrying
  }

  // Execute the trajectory if planning is successful
  if (plan_success) {
    // Time parameterization for velocity scaling
    robot_trajectory::RobotTrajectory robot_trajectory(move_group.getRobotModel(), PLANNING_GROUP);
    robot_trajectory.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);

    trajectory_processing::IterativeParabolicTimeParameterization time_param;
    bool success = time_param.computeTimeStamps(robot_trajectory, 0.05, 0.05); // Use scaling factors
    if (!success) {
      RCLCPP_ERROR(logger, "Time parameterization failed.");
    }

    robot_trajectory.getRobotTrajectoryMsg(trajectory);

    // Execute the trajectory if the fraction is above the threshold
    auto execution_result = move_group.execute(trajectory);

    if (execution_result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(logger, "Motion executed successfully.");
    } else {
        RCLCPP_ERROR(logger, "Motion execution failed.");
    }
  } else {
    RCLCPP_ERROR(logger, "Motion plan did not reach the desired fraction after %d attempts.", max_planning_attempts);
  }
  
  rclcpp::sleep_for(std::chrono::milliseconds(500));
  
  // Clear pose targets and remove collision objects
  move_group.clearPoseTargets();
  waypoints.clear();
  
  // Detach box3
  RCLCPP_INFO(LOGGER, "Detaching object: box3");
  move_group.detachObject("box3");

  // Detach box4
  RCLCPP_INFO(LOGGER, "Detaching object: box4");
  move_group.detachObject("box4");
  
  RCLCPP_INFO(LOGGER, "Remove the objects from the world");

  // List of object IDs to remove
  std::vector<std::string> object_ids;
  object_ids.push_back("box1");
  object_ids.push_back("box2");
  object_ids.push_back("box3");
  object_ids.push_back("box4");

  // Remove the objects from the planning scene
  planning_scene_interface.removeCollisionObjects(object_ids);

  
  /*Remove collision objects from planning scene
  std::vector<std::string> objects_to_remove = {"box1", "box2", "box3", "box4"};
  planning_scene_interface.removeCollisionObjects(objects_to_remove);*/
  
  rclcpp::sleep_for(std::chrono::milliseconds(500));
  move_group.stop();
  
  // Stop the executor and join the thread
  executor->cancel();
  if (spin_thread.joinable()) {
      spin_thread.join();
  }

  // Shutdown ROS
  RCLCPP_INFO(logger, "Shutting down ROS node.");
  rclcpp::shutdown();
  return 0;
}
