#! usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from sar_database_msgs.msg import WasteTarget
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        
        # Declare the namespace parameter
        self.declare_parameter('robot_namespace', '')
        
        # Storing the parameter
        self.ns = self.get_parameter('robot_namespace').value
        
        # Create the NavigateToPose ActionClient
        action_name = f'/{self.ns}/navigate_to_pose' if self.ns else 'navigate_to_pose'

        self.nav_to_pose_client = ActionClient(self, 
                                               NavigateToPose, 
                                               action_name)
        
        # Create the state publisher
        pub_topic = f'/{self.ns}/robot_state' if self.ns else 'robot_state'
        
        self.state_pub = self.create_publisher(String,
                                               pub_topic,
                                               10)
        
        # Create subscriber
        sub_topic = f'/{self.ns}/assigned_target' if self.ns else 'assigned_target'
        self.create_subscription(WasteTarget,
                                 sub_topic,
                                 self.assigned_target_callback,
                                 10)
        
        # Initialize the robot state
        self.waypoint_state = 0

        self.current_goal_handle = None
        self.current_nav_result = None
        self.assigned_target_pose = None
        
        # Create the waypoints list
        self.waypoints = [
            (0.0,0.0),
            (0.0,1.0),
            (1.0,0.0),
            (1.0,1.0),
        ]
        # Initialize waypoint index
        self.current_waypoint_idx = 0

        self.timer = self.create_timer(1.0, self.patrol_timer_callback)

        self.get_logger().info(f"WaypointNavigator initialised for namespace: {self.ns}")

    def patrol_timer_callback(self):
        if self.waypoint_state in [1,2]:
            return 

        if self.current_waypoint_idx >= len(self.waypoints):
            self.get_logger().info(f"All waypoints completed! Restarting patrol loop")
            self.current_waypoint_idx = 0

        x,y = self.waypoints[self.current_waypoint_idx]
        self.get_logger().info(
            f"Targetting waypoints {self.current_waypoint_idx}: {x},{y}")

        goal_msg = self.build_goal(x, y)

        self.waypoint_state = 1

        state_msg = String()
        state_msg.data = "navigating"
        self.state_pub.publish(state_msg)

        self.send_goal(goal_msg)
       
    def assigned_target_callback(self, msg):
        self.get_logger().info("Priority target recieved from allocator!")

        if self.current_goal_handle is not None:
            self.get_logger().info("Cancelling current patrol goal...")
            self.current_goal_handle.cancel_goal_async()

        self.assigned_target_pose = msg

        self.waypoint_state = 2

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = msg.position.x
        goal_msg.pose.pose.position.y = msg.position.y
        goal_msg.pose.pose.orientation.w = 1.0
        state_msg = String()
        state_msg.data = "assigned"
        # or "navigating" or "assigned"
        self.state_pub.publish(state_msg)
        self.send_goal(goal_msg)

    def build_goal(self, x, y):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation.w = 1.0

        return goal_msg

    def send_goal(self,goal_msg):
        self.get_logger().info(f"Waiting for the action server")

        if not self.nav_to_pose_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("Nav2 action server not available!")
            return
    
        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback = self.feedback_callback
        )

        send_goal_future.add_done_callback(
            self.goal_response_callback
        )
    
    def goal_response_callback(self,future):

        goal_handle = future.result() # Built in
        self.current_goal_handle = goal_handle

        if not goal_handle.accepted: # Built in
            self.get_logger().warn(f"Goal was rejected.")
            self.waypoint_state = 0
            state_msg = String()
            state_msg.data = "idle"
            self.state_pub.publish(state_msg)
            return
        
        self.get_logger().info(f"Goal was accepted.")
        state_msg = String()
        state_msg.data = "navigating"
        self.state_pub.publish(state_msg)
        result_future = goal_handle.get_result_async() # Built in
        # attatch another callback
        result_future.add_done_callback(
            self.result_callback)

    def feedback_callback(self, feedback_msg):

        feedback = feedback_msg.feedback

        self.get_logger().debug(
            f"Distance remaining: {feedback.distance_remaining:.2f}"
        )

    def result_callback(self, future):
        nav_result = future.result()

        self.current_nav_result = nav_result
        status = nav_result.status

        if status == 4:
            self.get_logger().info(f"Navigation completed successfully!")

            if self.waypoint_state == 1:
                self.current_waypoint_idx += 1
                self.waypoint_state = 0
                
            elif self.waypoint_state == 2:
                self.get_logger().info("Assigned target reached!")
                self.waypoint_state = 0

        else:
            self.get_logger().warn(f'Navigation failed. Statues {status}')

            self.waypoint_state = 0  
        # if not nav_result.accepted:
        #     self.get_logger.warn(f"Warning!, not accepted.")
        #     return
        # self.get_logger().info(f"Accepted.")

        # if nav_result is patrol_waypoint:
        state_msg = String()
        if status == 4:
            state_msg.data = "idle"
        else:
            state_msg.data = "idle"   # or "navigating" or "assigned"
        self.state_pub.publish(state_msg)

def main(args=None):
    rclpy.init(args=args)

    node = WaypointNavigator()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
