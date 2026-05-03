#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometric_shapes/check_isometry.h>


#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <thread>
#include <atomic>
#include <cmath> 
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Geometry>

#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <algorithm>
#include <cctype>
#include <tf2/LinearMath/Matrix3x3.h>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <chrono>
#include <iostream>
#include <filesystem>
#include <optional>

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

// Function to remove spaces and quotes from a string
std::string cleanString(const std::string& input) {
    std::string output = input;

    // Remove spaces
    output.erase(std::remove(output.begin(), output.end(), ' '), output.end());

    // Remove quotes
    output.erase(std::remove(output.begin(), output.end(), '"'), output.end());

    return output;
}

struct RunRecord {
  std::optional<long> time_01;
  std::optional<long> time_02;
  int row_index = -1;                // line number in the CSV (0-based, incl. header)
};

void log_time_value(
    long duration_ms,
    const std::string& filename = "/home/moniesha/Evaluation metrics/Efficiency/suctionnet/timings.csv")
{
  static const auto logger = rclcpp::get_logger("csv_logger");
  RCLCPP_INFO(logger, "➡️  log_time_value called | value=%ld ms", duration_ms);

  // --------------- ensure file & header ---------------
  const bool new_file = !std::filesystem::exists(filename);
  if (new_file) {
    RCLCPP_INFO(logger, "File %s does not exist — creating it with header", filename.c_str());
    std::ofstream f(filename);
    if (!f.is_open()) {
      RCLCPP_ERROR(logger, "❌ Failed to create %s", filename.c_str());
      return;
    }
    f << "time_01\n";  // Only one column now
    f.close();
  }

  // --------------- append the new value ---------------
  std::ofstream f(filename, std::ios::app);
  if (!f.is_open()) {
    RCLCPP_ERROR(logger, "❌ Cannot open %s for appending!", filename.c_str());
    return;
  }
  f << duration_ms << "\n";
  f.close();

  RCLCPP_INFO(logger, "✅ Logged time_01=%ld ms", duration_ms);
}

void broadcastGraspPose(const geometry_msgs::msg::Pose& grasp_pose, tf2_ros::StaticTransformBroadcaster& static_broadcaster)
{
    // Create the transform message
    geometry_msgs::msg::TransformStamped transform_stamped;
    
    // Set the header frame and timestamp
    transform_stamped.header.stamp = rclcpp::Clock().now();
    transform_stamped.header.frame_id = "base_link"; // Parent frame (change if needed)
    transform_stamped.child_frame_id = "grasp_frame"; // Frame for grasp_pose
    
    // Set translation and rotation from grasp_pose
    transform_stamped.transform.translation.x = grasp_pose.position.x;
    transform_stamped.transform.translation.y = grasp_pose.position.y;
    transform_stamped.transform.translation.z = grasp_pose.position.z;
    transform_stamped.transform.rotation.x = grasp_pose.orientation.x;
    transform_stamped.transform.rotation.y = grasp_pose.orientation.y;
    transform_stamped.transform.rotation.z = grasp_pose.orientation.z;
    transform_stamped.transform.rotation.w = grasp_pose.orientation.w;

    // Broadcast the transform
    static_broadcaster.sendTransform(transform_stamped);
}

struct GraspPoseWithScore {
  geometry_msgs::msg::Pose pose;
  float score;
};

GraspPoseWithScore convertAndPrintGraspDataToBaseLink(const std::string& file_path,const tf2_ros::Buffer& tf_buffer,int line_number){
    std::ifstream file(file_path);
    std::string line;
    std::string score_str;
    int row_number = 0;
    float S = 0.0;

    // Define variables to hold the translation and quaternion values
    std::string tx, ty, tz, qx, qy, qz, qw;

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string token;
        std::vector<std::string> row;

        // Split the line into tokens
        while (std::getline(ss, token, ',')) {
            row.push_back(token);
        }

        // Skip header or first row, you may adjust this depending on CSV format
        if (row_number++ == 0) continue;

        // Check for Grasp ID = 1
        if (row.size() > 0 && row[0] == std::to_string(line_number)) {
            // Clean the string values to remove spaces and quotes
            tx = cleanString(row[1]);
            ty = cleanString(row[2]);
            tz = cleanString(row[3]);
            qx = cleanString(row[4]);
            qy = cleanString(row[5]);
            qz = cleanString(row[6]);
            qw = cleanString(row[7]);
            score_str = cleanString(row[8]);
            S = std::stof(score_str);

            // Create a pose based on the tx, ty, tz, qx, qy, qz, qw values
            geometry_msgs::msg::Pose camera_pose;
            camera_pose.position.x = std::stod(tx);
            camera_pose.position.y = std::stod(ty);
            camera_pose.position.z = std::stod(tz);
            camera_pose.orientation.x = std::stod(qx);
            camera_pose.orientation.y = std::stod(qy);
            camera_pose.orientation.z = std::stod(qz);
            camera_pose.orientation.w = std::stod(qw);

            try {
                // Get the transform from camera_link to base_link
                geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer.lookupTransform("base_link", "camera_color_optical_frame", rclcpp::Time(0));

                // Apply the transform to the camera pose to convert to base_link
                geometry_msgs::msg::Pose grasp_pose;
                tf2::doTransform(camera_pose, grasp_pose, transform_stamped);

                rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("move_robot");
                tf2_ros::StaticTransformBroadcaster static_broadcaster(node);

                // Call the broadcast function
                broadcastGraspPose(grasp_pose, static_broadcaster);

                // Print the transformed values in base_link frame
                // RCLCPP_INFO(LOGGER, "Grasp ID: %s", row[0].c_str());
                // RCLCPP_INFO(LOGGER, "x = %s", tx.c_str());
                // RCLCPP_INFO(LOGGER, "y = %s", ty.c_str());
                // RCLCPP_INFO(LOGGER, "z = %s", tz.c_str());
                // RCLCPP_INFO(LOGGER, "qx = %s", qx.c_str());
                // RCLCPP_INFO(LOGGER, "qy = %s", qy.c_str());
                // RCLCPP_INFO(LOGGER, "qz = %s", qz.c_str());
                // RCLCPP_INFO(LOGGER, "qw = %s", qw.c_str());
                RCLCPP_INFO(LOGGER, "Transformed x = %.4f", grasp_pose.position.x);
                RCLCPP_INFO(LOGGER, "Transformed y = %.4f", grasp_pose.position.y);
                RCLCPP_INFO(LOGGER, "Transformed z = %.4f", grasp_pose.position.z);
                RCLCPP_INFO(LOGGER, "Transformed qx = %.4f", grasp_pose.orientation.x);
                RCLCPP_INFO(LOGGER, "Transformed qy = %.4f", grasp_pose.orientation.y);
                RCLCPP_INFO(LOGGER, "Transformed qz = %.4f", grasp_pose.orientation.z);
                RCLCPP_INFO(LOGGER, "Transformed qw = %.4f", grasp_pose.orientation.w);
                // Return the updated grasp_pose immediately after transformation
                return GraspPoseWithScore{grasp_pose, S};

            } catch (tf2::TransformException& ex) {
                RCLCPP_ERROR(LOGGER, "Transform error: %s", ex.what());
            }

            break;  // We found Grasp ID = 1, no need to continue reading
        }
    }
}

// Function to plan and execute a Cartesian path
bool planAndExecuteCartesianPath(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const geometry_msgs::msg::Pose& start_pose,
    const geometry_msgs::msg::Pose& target_pose,
    const std::string& planning_group) {
    RCLCPP_INFO(rclcpp::get_logger("CartesianPath"), "Planning Cartesian path...");

    // Create a planning scene interface object
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  // Correct declaration

    const double eef_step = 0.01;             // Step size for end-effector
    const double jump_threshold =2.0;         // Threshold to ignore jumps in IK solutions
    move_group.setPlanningTime(200.0);  // Increase planning time (in seconds)
    const int max_planning_attempts = 3;       // Maximum number of planning attempts (increased)

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(start_pose);

    // Add intermediate waypoints with interpolated orientations
    for (int i = 1; i <= 10; ++i) {
        double t = static_cast<double>(i) / 11.0;
        geometry_msgs::msg::Pose intermediate_pose;
        intermediate_pose.position.x = start_pose.position.x + i * (target_pose.position.x - start_pose.position.x) / 11;
        intermediate_pose.position.y = start_pose.position.y + i * (target_pose.position.y - start_pose.position.y) / 11;
        intermediate_pose.position.z = start_pose.position.z + i * (target_pose.position.z - start_pose.position.z) / 11;

        intermediate_pose.orientation = interpolateQuaternion(start_pose.orientation, target_pose.orientation, t);
        waypoints.push_back(intermediate_pose);
    }

    // Add the final pose
    waypoints.push_back(target_pose);

    rclcpp::sleep_for(std::chrono::milliseconds(500));

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = 0.0;
    bool plan_success = false;

    // Try multiple planning attempts

    for (int attempt = 0; attempt < max_planning_attempts; ++attempt) {
        fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        RCLCPP_INFO(rclcpp::get_logger("CartesianPath"), "Plan (Cartesian path) (%.2f%% achieved) on attempt %d", fraction * 100.0, attempt + 1);
        //0.91 default value
        if (fraction >= 0.87) {
            plan_success = true;
            break;
        }

        RCLCPP_WARN(rclcpp::get_logger("CartesianPath"), "Planning attempt %d failed. Trying again...", attempt + 1);
        rclcpp::sleep_for(std::chrono::milliseconds(500));
    }

    // Execute the trajectory if planning is successful
    if (plan_success) {
      // Time parameterization for velocity scaling
      robot_trajectory::RobotTrajectory robot_trajectory(move_group.getRobotModel(), planning_group);
      robot_trajectory.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);

      trajectory_processing::IterativeParabolicTimeParameterization time_param;
      bool success = time_param.computeTimeStamps(robot_trajectory, 0.25, 0.25); // Use scaling factors
      if (!success) {
        RCLCPP_ERROR(LOGGER, "Time parameterization failed.");
        return false;
      }

      robot_trajectory.getRobotTrajectoryMsg(trajectory);

      // Execute the trajectory if the fraction is above the threshold
      auto execution_result = move_group.execute(trajectory);

      if (execution_result == moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_INFO(LOGGER, "Motion executed successfully.");
          return true;
      } else {
          RCLCPP_ERROR(LOGGER, "Motion execution failed.");
          return false;
      }
    } else {
      RCLCPP_ERROR(LOGGER, "Collision detected. Motion plan did not reach the desired fraction");
      /*RCLCPP_ERROR(LOGGER, "Motion plan did not reach the desired fraction after %d attempts.", max_planning_attempts);*/
    }
  
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  
    // Clear pose targets and remove collision objects
    //move_group.clearPoseTargets();
    waypoints.clear();

    // // Detach box3
    // RCLCPP_INFO(LOGGER, "Detaching object: box3");
    // move_group.detachObject("box3");

    // // Detach box4
    // RCLCPP_INFO(LOGGER, "Detaching object: box4");
    // move_group.detachObject("box4");
  
    // RCLCPP_INFO(LOGGER, "Remove the objects from the world");

    // // List of object IDs to remove
    // std::vector<std::string> object_ids;
    // object_ids.push_back("box1");
    // object_ids.push_back("box2");
    // object_ids.push_back("box5");
    // object_ids.push_back("box3");
    // object_ids.push_back("box4");

    // // Remove the objects from the planning scene
    // planning_scene_interface.removeCollisionObjects(object_ids);

    // Ensure a return value in case of failure
    return false;
}

int main(int argc, char** argv)
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("suctin_move_robot", node_options);

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("suction_move_robot");

  // Create the MoveIt MoveGroup Interface
  static const std::string PLANNING_GROUP = "ur_manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

  // Create a publisher for the relay_control topic
  auto relay_pub = move_group_node->create_publisher<std_msgs::msg::String>("relay_control", rclcpp::QoS(10));
  auto sensor_pub = move_group_node->create_publisher<std_msgs::msg::String>("sensor_control", rclcpp::QoS(10));
  
  // Create TF2 buffer and listener
  tf2_ros::Buffer tf_buffer(move_group_node->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer);
  
  // Use a shared pointer for the executor
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(move_group_node);

  // Use a thread to spin the executor
  std::thread spin_thread([&executor]() { executor->spin(); });

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  std::chrono::high_resolution_clock::time_point start_1;

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
    box_pose1.position.y = 0.2;
    box_pose1.position.z = -0.288;

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

    // Create collision object for the robot to avoid (elevation box)
  auto const collision_object5 = [frame_id =
                                 move_group.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject collision_object5;
    collision_object5.header.frame_id = frame_id;
    collision_object5.id = "box5"; 
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 1.0;
    primitive.dimensions[primitive.BOX_Y] = 0.7;
    primitive.dimensions[primitive.BOX_Z] = 0.45;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose5;
    box_pose5.orientation.w = 1.0;  // We can leave out the x, y, and z components of the quaternion since they are initialized to 0
    box_pose5.position.x = 0;
    box_pose5.position.y = 0.85;
    box_pose5.position.z = -0.183;

    collision_object5.primitives.push_back(primitive);
    collision_object5.primitive_poses.push_back(box_pose5);
    collision_object5.operation = collision_object5.ADD;

    return collision_object5;
  }();  
    
  planning_scene_interface.applyCollisionObject(collision_object5);
    
  //Create collision object for the robot to avoid (gripper)
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
    primitive.dimensions[primitive.BOX_X] = 0.035;  // 0.090 for rg2 and fin-ray
    primitive.dimensions[primitive.BOX_Y] = 0.035; // 0.030 for rg2 and fin-ray 
    primitive.dimensions[primitive.BOX_Z] = 0.150; //Fin-Ray 0.221 (0.225 earlier), 0.190 for rg2 baseline

    // Define the pose of the collision object relative to wrist_3_link
    geometry_msgs::msg::Pose box_pose3;
    box_pose3.orientation.w = 1.0;  // No rotation (identity quaternion)
    box_pose3.position.x = 0.0;     // Adjust as needed
    box_pose3.position.y = 0.0;     // Adjust as needed
    box_pose3.position.z = 0.114;    // Center of the box along its height 

    // Add the shape and pose to the collision object
    attached_object.object.primitives.push_back(primitive);
    attached_object.object.primitive_poses.push_back(box_pose3);
    attached_object.object.operation = attached_object.object.ADD;

    return attached_object;
  }();  
   
  // Add the collision object to the scene (Camera)
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
    primitive.dimensions[primitive.BOX_X] = 0.140;  // 0.09 Size in meters
    primitive.dimensions[primitive.BOX_Y] = 0.135;
    primitive.dimensions[primitive.BOX_Z] = 0.090;    // 0.07

    // Define the pose of the collision object relative to wrist_3_link
    geometry_msgs::msg::Pose box_pose4;
    box_pose4.orientation.w = 1.0;  // No rotation (identity quaternion)
    box_pose4.position.x = -0.015;  // Adjust as needed 0
    box_pose4.position.y = 0.03;    // Adjust as needed
    box_pose4.position.z = 0.047;   // Center of the box along its height 0.039

    // Add the shape and pose to the collision object
    attached_object1.object.primitives.push_back(primitive);
    attached_object1.object.primitive_poses.push_back(box_pose4);
    attached_object1.object.operation = attached_object1.object.ADD;

    return attached_object1;
  }();  

  // Add the collision object to the scene
  planning_scene_interface.applyAttachedCollisionObject(attached_object1);

  move_group.clearPoseTargets();
  
  // Define the start and target poses for two movements
  geometry_msgs::msg::Pose start_pose_0, start_pose_1, target_pose_1, start_pose_2, target_pose_2a, target_pose_2, start_pose_3, target_pose_3, start_pose_4, target_pose_4, start_pose_5, target_pose_5, start_pose_6, target_pose_6, start_pose_7, target_pose_8;
  
  // //target_pose_6 = Home; 
  // target_pose_6.position.x = -0.12; //0.08
  // target_pose_6.position.y = 0.61; //0.65
  // target_pose_6.position.z = 0.63; //0.66
  // target_pose_6.orientation.x = 0;
  // target_pose_6.orientation.y = 1;
  // target_pose_6.orientation.z = 0;     
  // target_pose_6.orientation.w = 0; 

  //target_pose_6 = Home_new_14objects; 
  target_pose_6.position.x = -0.1279; //0.08
  target_pose_6.position.y = 0.5053; //0.65
  target_pose_6.position.z = 0.6660; //0.66
  target_pose_6.orientation.x = 0;
  target_pose_6.orientation.y = 1;
  target_pose_6.orientation.z = 0;     
  target_pose_6.orientation.w = 0;

  //target_pose_8 = Intermediate Home Pose
  target_pose_8.position.x = -0.1430;
  target_pose_8.position.y = 0.8352;
  target_pose_8.position.z = 0.5972;
  target_pose_8.orientation.x = 1;
  target_pose_8.orientation.y = 0;
  target_pose_8.orientation.z = 0;     
  target_pose_8.orientation.w = 0;

  //target_pose_5 = Drop pose 
  target_pose_5.position.x = 0.23;
  target_pose_5.position.y = 0.38;
  target_pose_5.position.z = 0.55;
  target_pose_5.orientation.x = 0;
  target_pose_5.orientation.y = 1;
  target_pose_5.orientation.z = 0;     
  target_pose_5.orientation.w = 0;

  // Assign current pose to start_pose_1
  start_pose_1 = move_group.getCurrentPose().pose;
  
  rclcpp::sleep_for(std::chrono::milliseconds(100));
  
  RCLCPP_INFO(LOGGER, "Start Pose 1 position: [%.4f, %.4f, %.4f]", 
              start_pose_1.position.x, start_pose_1.position.y, start_pose_1.position.z);
  RCLCPP_INFO(LOGGER, "Start Pose 1 orientation: [%.4f, %.4f, %.4f, %.4f]", 
              start_pose_1.orientation.x, start_pose_1.orientation.y, 
              start_pose_1.orientation.z, start_pose_1.orientation.w);

  bool success = false;
  int attempts = 1;
  // Try up to 3 times with different grasp poses
  while (attempts <= 5) {
    GraspPoseWithScore grasp_data = convertAndPrintGraspDataToBaseLink("/home/moniesha/suctionnet-baseline/neural_network/example_data/save_data/test_novel/scene_0160/realsense/suction/0000.csv", tf_buffer, attempts);  
    //target_pose_2a = convertAndPrintGraspDataToBaseLink("/home/moniesha/graspnet-baseline/doc/top_50_grasp_poses.csv", tf_buffer, attempts);
    target_pose_2a = grasp_data.pose; // Use the pose from the grasp data
    float score_2a = grasp_data.score; // Get the score from the grasp data 
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received pose with score: %.4f", score_2a);
 
      rclcpp::sleep_for(std::chrono::milliseconds(200));

      // Convert quaternion to rotation matrix
      tf2::Quaternion q_check(
          target_pose_2a.orientation.x,
          target_pose_2a.orientation.y,
          target_pose_2a.orientation.z,
          target_pose_2a.orientation.w);
      tf2::Matrix3x3 rotation_matrix_check(q_check);

      // Extract Z-axis of the grasp frame
      tf2::Vector3 z_axis_check = rotation_matrix_check.getColumn(2);

      // Compute the angle with respect to the XY plane of the base_link frame
      double angle_rad = std::atan2(std::sqrt(z_axis_check.x() * z_axis_check.x() + z_axis_check.y() * z_axis_check.y()), std::abs(z_axis_check.z()));
      double angle_deg = angle_rad * (180.0 / M_PI); // Convert to degrees

      // Check if the angle is at least 30 degrees (30 for fin-ray, 45 for rg2)
      if ((90-angle_deg) < 45.0 ) {
          RCLCPP_WARN(LOGGER, "Attempt %d rejected: Angle %.2f° is less than 30°. Trying next pose...", attempts, (90-angle_deg));
          attempts++;
          continue; // Skip the rest of the loop and try the next attempt
      }

      RCLCPP_INFO(LOGGER, "Attempt %d accepted: Angle %.2f° is valid.", attempts, (90-angle_deg));

      // The rest of your existing code remains unchanged

      // Desired distance to move backward along the Z-axis
      double distance1 = 0.150;//Adjusting gripper length (other 0.170, with fruit 178) for rg2 baseline gripper, for fin-ray 0.250 (0.254 initial))

      tf2::Quaternion q1(
          target_pose_2a.orientation.x,
          target_pose_2a.orientation.y,
          target_pose_2a.orientation.z,
          target_pose_2a.orientation.w);
      tf2::Matrix3x3 rotation_matrix1(q1);

      tf2::Vector3 z_axis1 = rotation_matrix1.getColumn(2); // Z-axis of the wrist3 frame
      tf2::Vector3 backward_vector1 = -distance1 * z_axis1;

      geometry_msgs::msg::Pose backward_pose1 = target_pose_2a; // Copy the original pose
      backward_pose1.position.x += backward_vector1.x();
      backward_pose1.position.y += backward_vector1.y();
      backward_pose1.position.z += backward_vector1.z();

      target_pose_2.position.x = backward_pose1.position.x;
      target_pose_2.position.y = backward_pose1.position.y;
      target_pose_2.position.z = backward_pose1.position.z;
      target_pose_2.orientation.x = target_pose_2a.orientation.x;
      target_pose_2.orientation.y = target_pose_2a.orientation.y;
      target_pose_2.orientation.z = target_pose_2a.orientation.z;
      target_pose_2.orientation.w = target_pose_2a.orientation.w;

      rclcpp::sleep_for(std::chrono::milliseconds(100));

      double distance2 = 0.20; // Example: 15 cm

      tf2::Quaternion q2(
          target_pose_2.orientation.x,
          target_pose_2.orientation.y,
          target_pose_2.orientation.z,
          target_pose_2.orientation.w);
      tf2::Matrix3x3 rotation_matrix2(q2);

      tf2::Vector3 z_axis2 = rotation_matrix2.getColumn(2); // Z-axis of the wrist3 frame
      tf2::Vector3 backward_vector2 = -distance2 * z_axis2;

      geometry_msgs::msg::Pose backward_pose2 = target_pose_2; // Copy the original pose
      backward_pose2.position.x += backward_vector2.x();
      backward_pose2.position.y += backward_vector2.y();
      backward_pose2.position.z += backward_vector2.z();

      target_pose_1.position.x = backward_pose2.position.x;
      target_pose_1.position.y = backward_pose2.position.y;
      target_pose_1.position.z = backward_pose2.position.z;
      target_pose_1.orientation.x = target_pose_2.orientation.x;
      target_pose_1.orientation.y = target_pose_2.orientation.y;
      target_pose_1.orientation.z = target_pose_2.orientation.z;
      target_pose_1.orientation.w = target_pose_2.orientation.w;

      // cycle starts
      start_1 = std::chrono::high_resolution_clock::now();
      success = planAndExecuteCartesianPath(move_group, start_pose_1, target_pose_1, PLANNING_GROUP);
      rclcpp::sleep_for(std::chrono::milliseconds(500));

      if (success) {
          RCLCPP_INFO(LOGGER, "Motion (home to Pre-grasp) executed successfully on attempt %d.", attempts);
          std_msgs::msg::String relay_msg;
          relay_msg.data = "ON";
          relay_pub->publish(relay_msg);
          RCLCPP_INFO(LOGGER, "Published 'suction on' to relay control. Waiting 2 seconds...");
          rclcpp::sleep_for(std::chrono::seconds(2));

          // ✅ Append attempt number and score to CSV file
          std::string output_file = "/home/moniesha/Evaluation metrics/grasp_quality_score/suctionnet/success_scores.csv";

          std::ofstream file(output_file, std::ios::app);  // open in append mode
          if (file.is_open()) {
            file << attempts << "," << score_2a << "\n";
            file.close();
            RCLCPP_INFO(LOGGER, "Appended Attempt: %d | Score: %.4f to %s", attempts, score_2a, output_file.c_str());
          } else {
            RCLCPP_ERROR(LOGGER, "Failed to open file: %s", output_file.c_str());
          }
          break;
      } else {
          RCLCPP_WARN(LOGGER, "Attempt %d failed. Trying next pose...", attempts);
          attempts++;
      }
  }

  if (!success) {
      RCLCPP_ERROR(LOGGER, "Motion (Home to Pre-Grasp) execution failed after 5 attempts.");
      return 1;
  }

  // touch node OFF
  std_msgs::msg::String sensor_msg;
  sensor_msg.data = "SOFF";
  sensor_pub->publish(sensor_msg);
  
  //moving from pre-grasp pose to grasp pose
  start_pose_2 = move_group.getCurrentPose().pose;
  rclcpp::sleep_for(std::chrono::milliseconds(100));
 
  // Plan and execute the second move (pre-grasp pose to grasp pose)
  if (planAndExecuteCartesianPath(move_group, start_pose_2, target_pose_2, PLANNING_GROUP)) {
    RCLCPP_INFO(LOGGER, "Move 2 (Pre-grasp to Grasp) execution completed.");

  } else {
    RCLCPP_ERROR(LOGGER, "Move 2 (Pre-grasp to Grasp) execution failed. Moving to home pose again");

    //Moving to home pose
    start_pose_0 = move_group.getCurrentPose().pose;
    rclcpp::sleep_for(std::chrono::milliseconds(100));

    if (planAndExecuteCartesianPath(move_group, start_pose_0, target_pose_6, PLANNING_GROUP)) { 
        RCLCPP_ERROR(LOGGER, "Moving to home pose.");
        // Publish "close" to custom_gripper topic and wait for 5 seconds
        std_msgs::msg::String relay_msg;
        relay_msg.data = "OFF";
        relay_pub->publish(relay_msg);
        RCLCPP_INFO(LOGGER, "Published 'suction off' to relay control. Waiting 2 seconds...");
        rclcpp::sleep_for(std::chrono::seconds(2));
    } else {
    RCLCPP_ERROR(LOGGER, "gripper closing failed.");
    }
    return 1;
  }
  
  //moving from grasp-pose to upward pose            
  start_pose_3 = move_group.getCurrentPose().pose;
  
  rclcpp::sleep_for(std::chrono::milliseconds(100));

  //calculating upward pose (15cm above on z axis)
  target_pose_3.position.x = start_pose_3.position.x; 
  target_pose_3.position.y = start_pose_3.position.y;
  target_pose_3.position.z = start_pose_3.position.z + 0.20;
  target_pose_3.orientation = start_pose_3.orientation;

  // Plan and execute the 3rd move (grasp-pose to upward-pose)
  if (planAndExecuteCartesianPath(move_group, start_pose_3, target_pose_3, PLANNING_GROUP)) {
    RCLCPP_INFO(LOGGER, "Move 3 (Grasp to upward-pose)execution completed.");
  } else {
    RCLCPP_ERROR(LOGGER, "Move 3 (Grasp to upward-pose) execution failed. Completing the cycle.");
    //moving to intermediate home pose
    start_pose_0 = move_group.getCurrentPose().pose;
    rclcpp::sleep_for(std::chrono::milliseconds(100));

    planAndExecuteCartesianPath(move_group, start_pose_0, target_pose_8, PLANNING_GROUP); 
    RCLCPP_INFO(LOGGER, "Moved to intermediate home pose.");
    rclcpp::sleep_for(std::chrono::milliseconds(100));

    // Moving to drop pose
    start_pose_5 = move_group.getCurrentPose().pose;
    rclcpp::sleep_for(std::chrono::milliseconds(100));

    if (planAndExecuteCartesianPath(move_group, start_pose_5, target_pose_5, PLANNING_GROUP)) {
          RCLCPP_INFO(LOGGER, "Moved to drop pose.");
          // Publish "open" to custom_gripper topic and wait for 5 seconds
          std_msgs::msg::String relay_msg;
          relay_msg.data = "OFF";
          relay_pub->publish(relay_msg);
          RCLCPP_INFO(LOGGER, "Published 'suction off' to relay_control. Waiting 2 seconds...");
          rclcpp::sleep_for(std::chrono::seconds(2));
    } else {
      RCLCPP_ERROR(LOGGER, "suction actuation failed.");
    }

    //moving to home pose
    start_pose_0 = move_group.getCurrentPose().pose;
    rclcpp::sleep_for(std::chrono::milliseconds(100));

    if (planAndExecuteCartesianPath(move_group, start_pose_0, target_pose_6, PLANNING_GROUP)) { 
        RCLCPP_ERROR(LOGGER, "Moving to home pose.");
        rclcpp::sleep_for(std::chrono::seconds(2));
    } else {
    RCLCPP_ERROR(LOGGER, "moving to home pose failed.");
    }

    return 1;
  }

  // Move to drop pose
  start_pose_5 = move_group.getCurrentPose().pose;
  
  // Plan and execute the 5th move (upward-pose to drop pose)
  if (planAndExecuteCartesianPath(move_group, start_pose_5, target_pose_5, PLANNING_GROUP)) {
    RCLCPP_INFO(LOGGER, "Move 5 (upward pose to drop pose) execution completed.");
    // Publish "open" to custom_gripper topic and wait for 5 seconds
        std_msgs::msg::String relay_msg;
          relay_msg.data = "OFF";
          relay_pub->publish(relay_msg);
          RCLCPP_INFO(LOGGER, "Published 'suction off' to relay_control. Waiting 3 seconds...");
          rclcpp::sleep_for(std::chrono::seconds(2));
  } else {
    RCLCPP_ERROR(LOGGER, "Move 5 (upward-pose to drop pose) execution failed.");
    //moving to intermediate home pose
    start_pose_0 = move_group.getCurrentPose().pose;
    rclcpp::sleep_for(std::chrono::milliseconds(100));

    planAndExecuteCartesianPath(move_group, start_pose_0, target_pose_8, PLANNING_GROUP); 
    RCLCPP_INFO(LOGGER, "Moved to intermediate home pose.");
    rclcpp::sleep_for(std::chrono::milliseconds(100));

    // Moving to drop pose
    start_pose_5 = move_group.getCurrentPose().pose;
    rclcpp::sleep_for(std::chrono::milliseconds(100));

    if (planAndExecuteCartesianPath(move_group, start_pose_5, target_pose_5, PLANNING_GROUP)) {
          RCLCPP_INFO(LOGGER, "Moved to drop pose.");
          // Publish "open" to custom_gripper topic and wait for 5 seconds
          std_msgs::msg::String relay_msg;
          relay_msg.data = "OFF";
          relay_pub->publish(relay_msg);
          RCLCPP_INFO(LOGGER, "Published 'suction off' to relay_control. Waiting 3 seconds...");
          rclcpp::sleep_for(std::chrono::seconds(2));
    } else {
      RCLCPP_ERROR(LOGGER, "scution deactuation failed.");
    }

    //moving to home pose
    start_pose_0 = move_group.getCurrentPose().pose;
    rclcpp::sleep_for(std::chrono::milliseconds(100));

    if (planAndExecuteCartesianPath(move_group, start_pose_0, target_pose_6, PLANNING_GROUP)) { 
        RCLCPP_ERROR(LOGGER, "Moving to home pose.");
        std_msgs::msg::String relay_msg;
        relay_msg.data = "OFF";
        relay_pub->publish(relay_msg);
        RCLCPP_INFO(LOGGER, "Published 'suction off' to relay_control. Waiting 3 seconds...");
        rclcpp::sleep_for(std::chrono::seconds(2));
    } else {
    RCLCPP_ERROR(LOGGER, "moving to home failed.");
    }
    return 1;
  }

  // touch node ON
  sensor_msg.data = "SON";
  sensor_pub->publish(sensor_msg);

  // Moving to home pose
  start_pose_7 = move_group.getCurrentPose().pose;
  
  // Plan and execute the 7th move (move 7)
  if (planAndExecuteCartesianPath(move_group, start_pose_7, target_pose_6, PLANNING_GROUP)) {
    RCLCPP_INFO(LOGGER, "Moving to home pose.");
        rclcpp::sleep_for(std::chrono::seconds(2));
  } else {
    RCLCPP_ERROR(LOGGER, "Moving to home pose failed.");
    return 1;
  }
  
  // cycle ends 
  auto end_1   = std::chrono::high_resolution_clock::now();
  auto dur_1 = std::chrono::duration_cast<std::chrono::milliseconds>(end_1 - start_1).count();
  RCLCPP_INFO(LOGGER, "Logging time_01: %ld ms", dur_1);
  log_time_value(dur_1);  // No col argument now!

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
  object_ids.push_back("box5");
  object_ids.push_back("box3");
  object_ids.push_back("box4");
  
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
