import rclpy
from rclpy.node import Node
import time
import random
import math

from sensor_msgs.msg import LaserScan, Image, Imu
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, Vector3
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState, ContactsState
from cv_bridge import CvBridge

# Constants
TIME_DELTA = 0.2
MAX_TIMESTEPS = 100
MAX_EPISODES = 10
COLLISION_THRESHOLD = 0.55

class LidarReader(Node):
    def __init__(self):
        super().__init__('lidar_reader')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        self.cur_scan = None

    def lidar_callback(self, msg):
        self.cur_scan = msg
        self.get_logger().info('Lidar Data Received')

class ImageReader(Node):
    def __init__(self):
        super().__init__('image_reader')
        self.subscription = self.create_subscription(
            Image,
            '/limo/depth_camera_link/image_raw',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.cur_image = None

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.cur_image = cv_image
            self.get_logger().info('Image Data Received')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

class ImuReader(Node):
    def __init__(self):
        super().__init__('imu_reader')
        self.subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )
        self.cur_imu = None

    def imu_callback(self, msg):
        self.cur_imu = msg
        self.get_logger().info('IMU Data Received')

class BumperCollisionDetector(Node):
    def __init__(self):
        super().__init__('bumper_collision_detector')
        self.subscription = self.create_subscription(
            ContactsState,
            'bumper_states',
            self.contact_callback,
            10
        )
        self.curr_contact = None

    def contact_callback(self, msg):
        # Save the complete contact states for later inspection.
        self.curr_contact = msg.states
        self.get_logger().info("Received contact states: " + str(self.curr_contact))


class TrainSimulation(Node):
    """ROS2 Node for simulation control with collision and flip checks."""
    def __init__(self):
        super().__init__('env')
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.unpause_client = self.create_client(Empty, "/unpause_physics")
        self.pause_client = self.create_client(Empty, "/pause_physics")
        self.set_model_state_client = self.create_client(SetEntityState, "/set_entity_state")
        
        # Instantiate sensor reader nodes.
        self.lidar_reader = LidarReader()
        self.image_reader = ImageReader()
        self.imu_reader = ImuReader()
        self.bumper_reader = BumperCollisionDetector()  # Bumper (contact sensor) node

    def step(self, action):
        # Publish the velocity command.
        vel_cmd = Twist()
        vel_cmd.linear.x = float(action[0])
        vel_cmd.angular.z = float(action[1])
        self.velocity_publisher.publish(vel_cmd)
        
        # Unpause physics so sensor topics update.
        while not self.unpause_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Unpause Service Not Available, Trying Again...')
        try:
            self.unpause_client.call_async(Empty.Request())
        except Exception as e:
            self.get_logger().error(f"Failed to call /unpause_physics: {e}")
        
        # Allow time for sensor updates.
        time.sleep(TIME_DELTA)
        
        # Check for collision using the bumper sensor state by inspecting curr_contact.
        rclpy.spin_once(self.bumper_reader, timeout_sec=0.2)
        collision = self._check_collision()
        
        # Check if the robot is flipped via the IMU.
        flipped = self._check_flipped()
        if flipped:
            self.get_logger().warn("Robot is flipped! Stopping the robot.")
        
        # Pause physics.
        while not self.pause_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Pause Service Not Available, Trying Again...')
        try:
            self.pause_client.call_async(Empty.Request())
        except Exception as e:
            self.get_logger().error(f"Failed to call /pause_physics: {e}")

        return collision or flipped

    def _set_state(self, model_name, pose, twist):
        # Wait for the service.
        while not self.set_model_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Set Model State Service Not Available, Trying Again...')
        model_state = EntityState()
        model_state.name = model_name
        model_state.pose = pose
        model_state.twist = twist
        request = SetEntityState.Request()
        request.state = model_state
        try:
            self.set_model_state_client.call_async(request)
        except Exception as e:
            self.get_logger().error(f"Failed to set model state: {e}")

    def _check_collision(self):
        """
        Check for a collision by inspecting the bumper sensor's curr_contact.
        Returns True if any contact is not with 'ground_plane::link::collision'.
        """
        rclpy.spin_once(self.bumper_reader, timeout_sec=0.2)
        if self.bumper_reader.curr_contact is not None and len(self.bumper_reader.curr_contact) > 0:
            for contact in self.bumper_reader.curr_contact:
                # Each contact has collision1_name and collision2_name
                if hasattr(contact, 'collision1_name') and hasattr(contact, 'collision2_name'):
                    if contact.collision1_name != 'ground_plane::link::collision' and contact.collision2_name != 'ground_plane::link::collision':
                        self.get_logger().warn("Collision detected via bumper sensor (not ground plane)!")
                        self.get_logger().info("Current contacts: " + str(self.bumper_reader.curr_contact))
                        return True
                    elif contact.collision1_name != 'ground_plane::link::collision' or contact.collision2_name != 'ground_plane::link::collision':
                        self.get_logger().warn("Collision detected via bumper sensor (not ground plane)!")
                        self.get_logger().info("Current contacts: " + str(self.bumper_reader.curr_contact))
                        return True
            # All contacts are with ground plane
            return False
        return False

    def _check_flipped(self):
        rclpy.spin_once(self.imu_reader, timeout_sec=0.2)
        if self.imu_reader.cur_imu is None:
            self.get_logger().warn("No IMU data received!")
            return False

        q = self.imu_reader.cur_imu.orientation
        # Convert quaternion to Euler angles.
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (q.w * q.y - q.z * q.x)
        pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
        self.get_logger().info(f"IMU Euler - Roll: {roll:.2f}, Pitch: {pitch:.2f}")
        if abs(roll) > math.pi / 2 or abs(pitch) > math.pi / 2:
            self.get_logger().warn("Robot appears to be flipped!")
            return True
        return False

    def reset(self):
        # Reset robot state.
        pose_robot = Pose()
        pose_robot.position = Point(x=0.0, y=0.0, z=0.1)
        pose_robot.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        twist_robot = Twist()
        twist_robot.linear = Vector3(x=0.0, y=0.0, z=0.0)
        twist_robot.angular = Vector3(x=0.0, y=0.0, z=0.0)
        self._set_state("limo_gazebosim", pose_robot, twist_robot)

        # Reset world model state.
        pose_world = Pose()
        pose_world.position = Point(x=-5.162810, y=-2.687440, z=0.0)
        pose_world.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        twist_world = Twist()
        twist_world.linear = Vector3(x=0.0, y=0.0, z=0.0)
        twist_world.angular = Vector3(x=0.0, y=0.0, z=0.0)
        self._set_state("simple", pose_world, twist_world)
        
        while not self.unpause_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Unpause Service Not Available, Trying Again...')
        try:
            self.unpause_client.call_async(Empty.Request())
        except Exception as e:
            self.get_logger().error(f"Failed to call /unpause_physics: {e}")
        time.sleep(TIME_DELTA)
        while not self.pause_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Pause Service Not Available, Trying Again...')
        try:
            self.pause_client.call_async(Empty.Request())
        except Exception as e:
            self.get_logger().error(f"Failed to call /pause_physics: {e}")

def main(args=None):
    rclpy.init(args=args)
    env = TrainSimulation()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(env)
    executor.add_node(env.lidar_reader)
    executor.add_node(env.image_reader)
    executor.add_node(env.imu_reader)
    executor.add_node(env.bumper_reader)

    try:
        for episode in range(MAX_EPISODES):
            env.reset()
            env.get_logger().info(f"Starting Episode: {episode}")
            done = False
            for timestep in range(MAX_TIMESTEPS):
                if done:
                    break
                linear_velocity = random.uniform(-5, 5)
                angular_velocity = random.uniform(-5, 5)
                done = env.step([linear_velocity, angular_velocity])
            env.get_logger().info(f"Episode {episode} completed.")
    except KeyboardInterrupt:
        pass
    finally:
        env.reset()
        executor.shutdown()
        env.destroy_node()
        env.lidar_reader.destroy_node()
        env.image_reader.destroy_node()
        env.imu_reader.destroy_node()
        env.bumper_reader.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
