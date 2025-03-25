from rclpy.node import Node
import rclpy
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, Vector3
from std_srvs.srv import Empty
import time
import random
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
import math


TIME_DELTA = 0.2
MAX_TIMESTEPS = 10
MAX_EPISODES = 10
GOAL_POSITION = [0.0, 0.0]
COLLISION_THRESHOLD=0.2

class LidarReader(Node):
    def __init__(self):
        super().__init__('lidar_reader')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)
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
            10)
        self.bridge = CvBridge()
        self.cur_image = None

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.cur_image = cv_image
        self.get_logger().info('Image Data Received')

class TrainSimulation(Node):
    """Superclass for all Gazebo Environments"""
    def __init__(self):
        super().__init__('env')
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.unpause = self.create_client(Empty, "/unpause_physics")
        self.pause = self.create_client(Empty, "/pause_physics")
        self.set_model_state = self.create_client(SetEntityState, "/set_entity_state")
        self.lidar_reader = LidarReader()
        self.image_reader = ImageReader()

    def step(self, action):
        # Execute Action in Gazebo
        vel_cmd = Twist()
        vel_cmd.linear.x = float(action[0])
        vel_cmd.angular.z = float(action[1])
        self.velocity_publisher.publish(vel_cmd)

        if self._check_collision():
            self.get_logger().info("Collision Detected! Stopping the robot.")
            return True
        
        while not self.unpause.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Unpause Service Not Available, Trying Again...')
        try:
            self.unpause.call_async(Empty.Request())
        except:
            print("Failed to call /unpause_physics")
        
        time.sleep(TIME_DELTA)

        while not self.pause.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Pause Service Not Available, Trying Again...')
        try:
            self.pause.call_async(Empty.Request())
        except (rclpy.ServiceException) as e:
            print("Failed to call /pause_physics")
        
        return False

    def _set_state(self, model_name, pose, twist):
        # Import required messages for setting model state
                
        while not self.set_model_state.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Set Model State Service Not Available, Trying Again...')
        
        # Create model state message with the provided parameters
        model_state = EntityState()
        model_state.name = model_name
        model_state.pose = pose
        model_state.twist = twist
        
        # Create and send request
        request = SetEntityState.Request()
        request.state = model_state
        
        try:
            self.set_model_state.call_async(request)
        except Exception as e:
            self.get_logger().error(f"Failed to set model state: {e}")
    
    def _check_collision(self):
        """Check if the robot is too close to any obstacle based on LIDAR data."""
        rclpy.spin_once(env.lidar_reader, timeout_sec=0.2)
        if self.lidar_reader.cur_scan is None:
            return False  # If LIDAR data is not available, no collision check
        
        # Check for obstacles in the scan range
        for range_val in self.lidar_reader.cur_scan.ranges:
            if range_val < COLLISION_THRESHOLD:  # If any range is below the threshold
                return True  # Collision detected
        return False  # No collision detected
        

    
    def reset(self):
        # Set Inital State of Robot
        pose = Pose()
        pose.position = Point(x=0.0, y=0.0, z=0.1)
        pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        twist = Twist()
        twist.linear = Vector3(x=0.0, y=0.0, z=0.0)
        twist.angular = Vector3(x=0.0, y=0.0, z=0.0)
        self._set_state("limo_gazebosim", pose, twist)

        # Set Inital State of World Model
        pose = Pose()
        pose.position = Point(x=-5.162810, y=-2.687440, z=0.0)
        pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        twist = Twist()
        twist.linear = Vector3(x=0.0, y=0.0, z=0.0)
        twist.angular = Vector3(x=0.0, y=0.0, z=0.0)
        self._set_state("simple", pose, twist)

        

        

        while not self.unpause.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Unpause Service Not Available, Trying Again...')
        try:
            self.unpause.call_async(Empty.Request())
        except:
            print("Failed to call /unpause_physics")
        
        time.sleep(TIME_DELTA)

        while not self.pause.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Pause Service Not Available, Trying Again...')
        try:
            self.pause.call_async(Empty.Request())
        except (rclpy.ServiceException) as e:
            print("Failed to call /pause_physics")
        

    

if "__main__" == __name__:
    rclpy.init(args=None)
    
    # Initalize the Environment
    env = TrainSimulation()
    
    try:
        for episode in range(MAX_EPISODES):
            # Reset the Environment
            env.reset()
            env.get_logger().info(f"Episode: {episode}")
            done = False
            for timestep in range(MAX_TIMESTEPS):
                if done:
                    break

                



                # rclpy.spin_once(env.image_reader, timeout_sec=0.2)
                # image_data = env.image_reader.cur_image
                # print(type(image_data))
                
                # Sudo-Neural Network Logic
                linear_velocity = random.uniform(1, 2)
                angular_velocity = random.uniform(1, 100)

                # Perform a step with the randomized action
                done = env.step([linear_velocity, angular_velocity])


             

    except KeyboardInterrupt:
        pass
    # Reset the environment before shutting down
    env.reset()
    rclpy.shutdown()