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
from nav_msgs.msg import Odometry
from PPOAgent import PPOAgent

# Constants
TIME_DELTA = 0.2
MAX_TIMESTEPS = 50
MAX_EPISODES = 500
COLLISION_THRESHOLD = 0.55

GOAL = [6.0, -2.0]
# GOAL = [0, 0]


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

class OdomReader(Node):
    def __init__(self):
        super().__init__('odom_reader')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.cur_odom = None

    def odom_callback(self, msg):
        self.cur_odom = msg

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
        self.odom_reader = OdomReader()
        # Four bumper/contact sensor nodes for each wheel
        self.front_left_bumper = BumperCollisionDetector('/front_left_wheel_contact', 'front_left_bumper')
        self.front_right_bumper = BumperCollisionDetector('/front_right_wheel_contact', 'front_right_bumper')
        self.rear_left_bumper = BumperCollisionDetector('/rear_left_wheel_contact', 'rear_left_bumper')
        self.rear_right_bumper = BumperCollisionDetector('/rear_right_wheel_contact', 'rear_right_bumper')

    def get_distance_and_angle_to_goal(self, goal=GOAL):
        """
        Returns (distance, angle) from current position to goal.
        Angle is relative to robot's heading (in radians).
        """

        rclpy.spin_once(self.odom_reader, timeout_sec=0.1)

        odom = self.odom_reader.cur_odom
        if odom is None:
            self.get_logger().warn("No odometry data received yet!")
            return None, None
        # Current position
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        # Goal position
        goal_x, goal_y = goal
        # Distance
        distance = math.hypot(goal_x - x, goal_y - y)
        # Robot orientation (yaw)
        q = odom.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        # Angle to goal
        angle_to_goal = math.atan2(goal_y - y, goal_x - x)
        # Relative angle
        angle = angle_to_goal - yaw
        # Normalize angle to [-pi, pi]
        angle = (angle + math.pi) % (2 * math.pi) - math.pi
        return distance, angle

    def get_lidar_data(self):
        rclpy.spin_once(self.lidar_reader, timeout_sec=0.1)
        scan = self.lidar_reader.cur_scan
        if scan is None:
            self.get_logger().warn("No LIDAR data received!")
            return None
        return list(scan.ranges)

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
    executor.add_node(env.odom_reader)
    executor.add_node(env.front_left_bumper)
    executor.add_node(env.front_right_bumper)
    executor.add_node(env.rear_left_bumper)
    executor.add_node(env.rear_right_bumper)

    # Initialize PPO agent with state_dim and action_dim
    # Get initial lidar size
    initial_lidar = env.get_lidar_data() or []
    state_dim = 2 + len(initial_lidar)
    action_dim = 2
    agent = PPOAgent(state_dim, action_dim)

    try:
        for episode in range(MAX_EPISODES):
            env.reset()
            env.get_logger().info(f"Starting Episode: {episode}")

            # initialize prev distance
            prev_dist, _ = env.get_distance_and_angle_to_goal()
            episode_reward = 0.0
            for timestep in range(MAX_TIMESTEPS):
                # get current state
                dist, ang = env.get_distance_and_angle_to_goal()
                lidar = env.get_lidar_data() or []
                state = [dist, ang] + lidar

                # select action from agent
                action = agent.select_action(state)

                # perform step in environment
                done = env.step(action)

                # compute reward: encourage reducing distance
                new_dist, _ = env.get_distance_and_angle_to_goal()
                reward = prev_dist - new_dist
                # angle-based reward shaping
                reward += 0.1 * (1 - abs(ang) / math.pi)
                # obstacle avoidance penalty
                if lidar:
                    min_range = min(lidar)
                    if min_range < 0.3:
                        reward -= 0.5 * (0.3 - min_range)
                # time step penalty to encourage faster completion
                reward -= 0.01
                # big reward for reaching goal
                if new_dist < 0.5:
                    reward += 100.0
                    done = True
                # penalty for collision or flip
                if done and new_dist >= 0.5:
                    reward -= 10.0

                agent.store_reward(reward, done)
                episode_reward += reward
                prev_dist = new_dist

                if done:
                    break

            # update PPO agent after each episode
            agent.update()
            env.get_logger().info(f"Episode {episode} completed. Total Reward: {episode_reward:.2f}")
    except KeyboardInterrupt:
        pass
    finally:
        env.reset()
        executor.shutdown()
        env.destroy_node()
        env.lidar_reader.destroy_node()
        env.image_reader.destroy_node()
        env.imu_reader.destroy_node()
        env.odom_reader.destroy_node()
        env.front_left_bumper.destroy_node()
        env.front_right_bumper.destroy_node()
        env.rear_left_bumper.destroy_node()
        env.rear_right_bumper.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
