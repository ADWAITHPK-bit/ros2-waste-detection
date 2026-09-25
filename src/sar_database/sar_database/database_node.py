#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import csv
import json
from sar_database_msgs.msg import WasteTarget
from std_srvs.srv import Trigger

class DatabaseNode(Node):
    def __init__(self):
        super().__init__('database_node')
        # Empty dictionary for detections
        self.targets = {}

        # Create subscription
        self.create_subscription(
            WasteTarget,'/detections',
            self.detection_callback,
            10)

        # Crete publisher
        self.detect_pub = self.create_publisher(
            String,'/target_list',
            10)

        # Create service
        self.get_target_srv = self.create_service(
            Trigger,
            '/get_targets',
            self.get_targets_callback
        )

        self.create_timer(2.0,self.publish_target_list)

        with open("database_log.csv", "w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow([
                "marker_id",
                "x",
                "y",
                "robot",
                "timestamp",
                "assigned",
                "assigned_to"
            ])

    def detection_callback(self, msg):

        marker_id = msg.marker_id
        x = msg.position.x
        y = msg.position.y
        robot = msg.detected_by
        timestamp = msg.timestamp

        if marker_id in self.targets:
            self.get_logger().info(f"Duplicate target ignored.")
            return
        
        self.targets[marker_id] = {
                "x" : x,
                "y" : y,
                "robot" : robot,
                "time" : str(timestamp),
                "assigned" : msg.assigned,
                "assigned_to" : msg.assigned_to
            }

         # Save JSON
        self.save_json()

        # Append one row to CSV
        self.append_csv({
            "marker_id": marker_id,
            "x": x,
            "y": y,
            "robot": robot,
            "time": str(timestamp),
            "assigned": msg.assigned,
            "assigned_to": msg.assigned_to
        })
        # Number of targets stored
        self.get_logger().info(f"{len(self.targets)} currently stored.")

    def get_targets_callback(self, request, response):
        # Convert dict to JSON
        json_string = json.dumps(self.targets, indent = 4)
        # Json string to response.message
        response.message = json_string
        response.success = True
        return response
    
    def publish_target_list(self):
        json_string = json.dumps(self.targets, indent = 4)

        msg = String()

        msg.data = json_string

        self.detect_pub.publish(msg)

        self.get_logger().info(
            f"Published {len(self.targets)} targets."
        )

    def save_json(self):
        with open("database_log.json", "w", encoding="utf-8")as file:
            json.dump(self.targets, file, indent=4)

    def append_csv(self, target):
        with open('database_log.csv',mode='a', newline='') as file:
            csv_writer = csv.writer(file)
            
            csv_writer.writerow([
                target["marker_id"],
                target["x"],
                target["y"],
                target["robot"],
                target["time"],
                target["assigned"],
                target["assigned_to"]
            ])

def main(args=None):
    rclpy.init(args=args)
    node = DatabaseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()