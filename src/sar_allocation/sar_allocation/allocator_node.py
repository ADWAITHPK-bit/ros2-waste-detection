#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from sar_database_msgs.msg import WasteTarget
import json
import math

class AllocatorNode(Node):

    def __init__(self):
        super().__init__("allocator_node")
        
        self.declare_parameter("robot_namespace", "")
        ns = self.get_parameter("robot_namespace").value
        # Dictionary to store the latest position of the robot
        self.robot_positions = {}
        self.robot_states = {}

        # Subscription to robot odometry
        self.create_subscription(
            Odometry,
            f'/{ns}/odom' if ns else '/odom',
            self.robot1_odom_callback,
            10
        )

        self.create_subscription(
            Odometry,
            '/robot2/odom',
            self.robot2_odom_callback,
            10
        )        

        # To robot state
        self.create_subscription(
            String,
            f'/{ns}/robot_state' if ns else '/robot_state',
            self.robot1_state_callback,
            10
        )

        self.create_subscription(
            String,
            '/robot2/robot_state',
            self.robot2_state_callback,
            10
        )

        # Service
        # "Database, can you give me all the targets?"
        self.get_target_client = self.create_client(
            Trigger,
            '/get_targets'
        )

        # Publisher for the robots 
        self.robot1_pub = self.create_publisher(
            WasteTarget,
            f'/{ns}/assigned_target' if ns else '/assigned_target',
            10
        )

        self.robot2_pub = self.create_publisher(
            WasteTarget,
            '/robot2/assigned_target',
            10
        )

        self.time_period = 3.0

        self.timer = self.create_timer(
            self.time_period, 
            self.allocation_timer_callback
        )

    def robot1_odom_callback(self, msg):
        self.robot1_x = msg.pose.pose.position.x
        self.robot1_y = msg.pose.pose.position.y
        self.robot_positions["robot1"] = (self.robot1_x, self.robot1_y)

    def robot2_odom_callback(self, msg):
        self.robot2_x = msg.pose.pose.position.x
        self.robot2_y = msg.pose.pose.position.y
        self.robot_positions["robot2"] = (self.robot2_x, self.robot2_y)

    def robot1_state_callback(self, msg):
        self.robot_states["robot1"] = msg.data

    def robot2_state_callback(self, msg):
        self.robot_states["robot2"] = msg.data

    def allocation_timer_callback(self):
        
        if not self.get_target_client.service_is_ready():
            self.get_logger().info("Waiting for the service.")
            return None
        # An empty trigger request
        request = Trigger.Request()

        # Call the services asynchronously
        self.future = self.get_target_client.call_async(request)
        # Register other callback that will recieve the database
        self.future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        # Obtaining the service response
        response = future.result()

        if not response.success:
            self.get_logger().warn("Warning! service not succeeded.")
            return
        
        targets = json.loads(response.message)

        self.allocate_targets(targets)

    def allocate_targets(self, targets):
        for target_id, target_data in targets.items():
            if target_data.get("assigned", False) == True:
                continue

            idle_robots = []
            # self.robot_states syntax: {"robot1": (idle_bool, assigned_bool, navigating_bool)}
            for robot_id, state_tuple in self.robot_states.items():
                if self.robot_states.get(robot_id) == "idle": # Index of the 'idle'
                    idle_robots.append(robot_id)

            if not idle_robots:
                self.get_logger().info("No idle robots left. Stopping allocation ...")
                break

            # Keep track of the smallest distance, and which robot achieved it
            closest_robot = None
            min_distance = float('inf')

            target_x = target_data["x"]
            target_y = target_data["y"]

            for robot_id in idle_robots:
                # Get current (x, y) coordinates of the robot from self.robot_positions
                if robot_id in self.robot_positions:
                    robot_x, robot_y = self.robot_positions[robot_id]
                    # Compute distance
                    dist = self.compute_distance(robot_x, robot_y, target_x, target_y)

                    # Track the min distance
                    if dist < min_distance:
                        min_distance = dist
                        closest_robot = robot_id

            # Publish the target to that robot
            # if closest_robot is not None:
            #     msg = WasteTarget()
            #     msg.position.x = float(target_x)
            #     msg.position.y = float(target_y)

            #     if closest_robot == 'robot1':
            #         self.robot1_pub.publish(msg)
            #     elif closest_robot == 'robot2':
            #         self.robot2_pub.publish(msg)

            #     # Marking as the robot as assigned locally, to prevent 
            #     # giving it another target
            #     # Update the state tuple to (idle=False, assigned=True, navigating=False)
            #     self.robot_states[closest_robot] = (False, True, False)

            if closest_robot is not None:
                self.publish_assignment(closest_robot, target_data)

                self.robot_states[closest_robot] = "assigned"

    def compute_distance(self,x1,y1,x2,y2):
        return math.sqrt((x2-x1)**2+(y2-y1)**2)

    def publish_assignment(self, robot_name, target):
        msg = WasteTarget()
        # Reading from the dictionary target using ["key"] 
        # and assigning it to the message property field
        msg.position.x = float(target["x"])
        msg.position.y = float(target["y"])

        if robot_name == 'robot1':
            publisher = self.robot1_pub
        else:
            publisher = self.robot2_pub

        publisher.publish(msg)

        self.get_logger().info(f"Target at ({msg.position.x}, {msg.position.y}) was assigned to {robot_name}")

def main(args=None):
    rclpy.init(args=args)

    node = AllocatorNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
    