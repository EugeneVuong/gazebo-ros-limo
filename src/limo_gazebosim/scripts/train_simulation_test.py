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
    def __init__(self, topic_name, node_name):
        super().__init__(node_name)
        self.subscription = self.create_subscription(
            ContactsState,
            topic_name,
            self.contact_callback,
            10
        )
        self.curr_contact = None

    def contact_callback(self, msg):
        self.curr_contact = msg.states

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
        # Four bumper/contact sensor nodes for each wheel
        self.front_left_bumper = BumperCollisionDetector('/front_left_wheel_contact', 'front_left_bumper')
        self.front_right_bumper = BumperCollisionDetector('/front_right_wheel_contact', 'front_right_bumper')
        self.rear_left_bumper = BumperCollisionDetector('/rear_left_wheel_contact', 'rear_left_bumper')
        self.rear_right_bumper = BumperCollisionDetector('/rear_right_wheel_contact', 'rear_right_bumper')

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
        
        # Check for collision using all four bumper sensor states by inspecting curr_contact.
        rclpy.spin_once(self.front_left_bumper, timeout_sec=0.2)
        rclpy.spin_once(self.front_right_bumper, timeout_sec=0.2)
        rclpy.spin_once(self.rear_left_bumper, timeout_sec=0.2)
        rclpy.spin_once(self.rear_right_bumper, timeout_sec=0.2)
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
        Check for a collision by inspecting all four wheel bumper sensors' curr_contact.
        Returns True if collision detected with anything except ground plane.
        Returns False if collision is with ground plane or no collision detected.
        """
        bumpers = [
            self.front_left_bumper,
            self.front_right_bumper,
            self.rear_left_bumper,
            self.rear_right_bumper
        ]
        for bumper in bumpers:
            rclpy.spin_once(bumper, timeout_sec=0.2)
            if bumper.curr_contact is None or len(bumper.curr_contact) == 0:
                continue  # No collision detected for this bumper
            for contact in bumper.curr_contact:
                if hasattr(contact, 'collision1_name') and hasattr(contact, 'collision2_name'):
                    # If either collision is with ground plane, this contact is not a real collision
                    if contact.collision1_name == 'ground_plane::link::collision' or contact.collision2_name == 'ground_plane::link::collision':
                        continue
                    else:
                        self.get_logger().warn(f"Collision detected via {bumper.get_name()}!")
                        print(f"Collision1: {contact.collision1_name}, Collision2: {contact.collision2_name}")
                        return True
        return False  # No non-ground collisions found

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
        # Use the /reset_simulation service to reset the simulation.
        reset_client = self.create_client(Empty, "/reset_simulation")
        while not reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Reset Simulation Service Not Available, Trying Again...')
        try:
            reset_client.call_async(Empty.Request())
        except Exception as e:
            self.get_logger().error(f"Failed to call /reset_simulation: {e}")
        time.sleep(TIME_DELTA)

def main(args=None):
    rclpy.init(args=args)
    env = TrainSimulation()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(env)
    executor.add_node(env.lidar_reader)
    executor.add_node(env.image_reader)
    executor.add_node(env.imu_reader)
    executor.add_node(env.front_left_bumper)
    executor.add_node(env.front_right_bumper)
    executor.add_node(env.rear_left_bumper)
    executor.add_node(env.rear_right_bumper)

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
        env.front_left_bumper.destroy_node()
        env.front_right_bumper.destroy_node()
        env.rear_left_bumper.destroy_node()
        env.rear_right_bumper.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
