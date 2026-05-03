import rclpy
from rclpy.node import Node
import os
import subprocess
import scipy.io as scio
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from PIL import Image as PILImage
import cv2

class RealSenseDataSaver(Node):
    def __init__(self):
        super().__init__('realsense_data_saver')

        # ROS 2 subscriptions
        self.color_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.color_callback,
            10
        )
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.camera_info_callback,
            10
        )

        # CV Bridge for converting ROS2 Image messages to OpenCV images
        self.bridge = CvBridge()

        # Storage for images, camera info, and grasp pose data
        self.color_image = None
        self.depth_image = None
        self.intrinsic_matrix = None
        self.grasp_pose_data = None  # Cache for Grasp ID == 1 data

        self.inference_run = False  # Flag to check if inference_command.sh has run
        self.grasp_processed = False  # Flag to indicate if Grasp ID 1 has been processed

        # Path to save images
        self.save_path_color = '/home/moniesha/suctionnet-baseline/neural_network/example_data/test_novel/scenes/scene_0160/realsense/rgb'
        self.save_path_depth = '/home/moniesha/suctionnet-baseline/neural_network/example_data/test_novel/scenes/scene_0160/realsense/depth'
        self.save_path_meta = '/home/moniesha/suctionnet-baseline/neural_network/example_data/test_novel/scenes/scene_0160/realsense/meta'

    def camera_info_callback(self, msg):
        if self.grasp_processed:
            return  # Skip further processing if Grasp ID 1 has already been processed

        self.intrinsic_matrix = np.array([
            [msg.k[0], 0.0, msg.k[2]],
            [0.0, msg.k[4], msg.k[5]],
            [0.0, 0.0, 1.0]
        ])
        self.get_logger().info(f"Intrinsic matrix received: {self.intrinsic_matrix}")

    def color_callback(self, msg):
        if self.grasp_processed:
            return  # Skip further processing if Grasp ID 1 has already been processed

        if self.intrinsic_matrix is None:
            self.get_logger().warn("Intrinsic matrix not yet available. Skipping color image processing.")
            return

        try:
            self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            color_image_path = os.path.join(self.save_path_color, '0000.png')
            PILImage.fromarray(cv2.cvtColor(self.color_image, cv2.COLOR_BGR2RGB)).save(color_image_path)
            self.get_logger().info(f"Color image saved at: {color_image_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to process color image: {e}")

    def depth_callback(self, msg):
        if self.grasp_processed:
            return  # Skip further processing if Grasp ID 1 has already been processed

        if self.intrinsic_matrix is None:
            self.get_logger().warn("Intrinsic matrix not yet available. Skipping depth image processing.")
            return

        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            depth_image_path = os.path.join(self.save_path_depth, '0000.png')
            PILImage.fromarray(self.depth_image).save(depth_image_path)
            self.get_logger().info(f"Depth image saved at: {depth_image_path}")
            self.create_meta_file()
            self.verify_and_run_inference()
        except Exception as e:
            self.get_logger().error(f"Failed to process depth image: {e}")

    def create_meta_file(self):
        if self.intrinsic_matrix is not None:
            factor_depth = 1000.0
            meta_data = {
                'intrinsic_matrix': self.intrinsic_matrix,
                'factor_depth': factor_depth
            }
            meta_file_path = os.path.join(self.save_path_meta, '0000.mat')
            scio.savemat(meta_file_path, meta_data)
            self.get_logger().info(f"Meta file saved at: {meta_file_path}")
        else:
            self.get_logger().warn("Intrinsic matrix is not available yet. Meta file not created.")

    def verify_and_run_inference(self):
        if self.inference_run or self.grasp_processed:
            return  # Skip inference if it has already run or Grasp ID 1 is processed

        color_image_path = os.path.join(self.save_path_color, '0000.png')
        depth_image_path = os.path.join(self.save_path_depth, '0000.png')
        meta_file_path = os.path.join(self.save_path_meta, '0000.mat')

        if os.path.exists(color_image_path) and os.path.exists(depth_image_path) and os.path.exists(meta_file_path):
            self.get_logger().info("All required files are successfully saved. Proceeding to run inference_command.sh.")
            inference_script_path = '/home/moniesha/grasp_benchmark_ws/src/suction_grasp/suction_grasp/inference_command.sh'

            try:
                subprocess.run(['bash', inference_script_path], check=True)
                self.get_logger().info("inference_command.sh script executed successfully.")
                self.inference_run = True  # Mark inference as run
                self.grasp_processed = True  # Mark Grasp ID 1 as processed
            except subprocess.CalledProcessError as e:
                self.get_logger().error(f"Failed to run inference_command.sh: {e}")
        else:
            self.get_logger().warn("Required files are not found. Skipping inference_command.sh execution.")


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseDataSaver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down RealSense Data Saver Node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

