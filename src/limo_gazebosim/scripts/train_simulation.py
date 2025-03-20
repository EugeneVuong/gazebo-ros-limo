from rclpy.node import Node
import rclpy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
import time
import random
TIME_DELTA = 0.2
MAX_TIMESTEPS = 5
MAX_EPISODES = 10

class TrainSimulation(Node):
    """Superclass for all Gazebo Environments"""
    def __init__(self):
        super().__init__('env')
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.unpause = self.create_client(Empty, "/unpause_physics")
        self.pause = self.create_client(Empty, "/pause_physics")
        self.reset_proxy = self.create_client(Empty, "/reset_world")

    def step(self, action):
        # Execute Action in Gazebo
        vel_cmd = Twist()
        vel_cmd.linear.x = float(action[0])
        vel_cmd.angular.z = float(action[1])
        self.velocity_publisher.publish(vel_cmd)
        
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


    
    def reset(self):
        while not self.reset_proxy.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Reset Service Not Available, Trying Again...')
        try:
            self.reset_proxy.call_async(Empty.Request())
            self.get_logger().info('Reset Service Called')
        except:
            print("Failed to call /reset_world")

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
            
            for timestep in range(MAX_TIMESTEPS):
                
                # Sudo-Neural Network Logic
                linear_velocity = random.uniform(1, 100)
                angular_velocity = random.uniform(1, 100)

                # Perform a step with the randomized action
                env.step([linear_velocity, angular_velocity])

    except KeyboardInterrupt:
        pass
    # Reset the environment before shutting down
    rclpy.shutdown()