import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import threading
import time
import pymysql

class ROS2NodeBridge(Node):
    """
    Bridges ROS2 topics with the Flask application.
    Handles data subscription and periodic database logging.
    """
    def __init__(self):
        super().__init__('flask_bridge')
        self.light = 0
        self.threshold = 768
        self.status = "Monitoring"
        
        # ROS2 Interface
        self.pub = self.create_publisher(Int32, 'set_threshold', 10)
        self.create_subscription(Int32, 'light_level', self.update_light, 10)
        
        # Database Connection
        self.db_conn = pymysql.connect(host='127.0.0.1', user='root', password='', database='light_monitor', autocommit=True)
        self.last_log = time.time()

    def update_light(self, msg):
        self.light = msg.data
        if time.time() - self.last_log > 5:
            self.log_db()

    def log_db(self):
        try:
            with self.db_conn.cursor() as cur:
                # Log sensor reading and actuator state
                cur.execute("INSERT INTO light_readings (light_level, threshold, led_state) VALUES (%s, %s, %s)",
                           (self.light, self.threshold, self.light < self.threshold))
            self.last_log = time.time()
        except pymysql.Error:
            pass # Suppress transient DB errors

    def set_thresh(self, val):
        self.threshold = val
        self.pub.publish(Int32(data=val))
        return True

class BridgeRunner:
    """Manages the ROS2 node execution thread."""
    def __init__(self):
        self.node = None
    
    def start(self):
        threading.Thread(target=self.run, daemon=True).start()
    
    def run(self):
        rclpy.init()
        self.node = ROS2NodeBridge()
        rclpy.spin(self.node)

    def get_data(self):
        if not self.node: return {'light_level': 0, 'threshold': 768, 'status': 'Starting'}
        return {'light_level': self.node.light, 'threshold': self.node.threshold, 'status': self.node.status}

    def set_threshold(self, val):
        if self.node: return self.node.set_thresh(val)
        return False

ros2_bridge = BridgeRunner()
