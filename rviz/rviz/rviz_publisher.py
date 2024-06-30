import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from ro45_portalrobot_interfaces.msg import RobotPos
from object_interfaces.msg import ObjectData
from std_msgs.msg import Bool, Float64
import time

class RVizPublisher(Node):
    def __init__(self):
        super().__init__('rviz_publisher')
        self.robot_position_sub = self.create_subscription(RobotPos, 'robot_position', self.robot_position_callback, 10)
        self.object_data_sub = self.create_subscription(ObjectData, 'object_data', self.object_data_callback, 10)
        self.init_bool_sub = self.create_subscription(Bool, 'init_bool', self.init_bool_callback, 10)
        self.velocity_sub = self.create_subscription(Float64, 'velocity', self.velocity_callback, 10)
        self.robot_marker_pub = self.create_publisher(Marker, 'rviz_robot_marker', 10)
        self.object_marker_pub = self.create_publisher(Marker, 'rviz_object_marker', 10)
        self.velocity_marker_pub = self.create_publisher(Marker, 'rviz_velocity_marker', 10)
        self.conveyor_belt_marker_pub = self.create_publisher(Marker, 'rviz_conveyor_belt_marker', 10)
        self.zero_position = {'x': None, 'y': None, 'z': None}  
        self.zero_position_set = False
        self.objects = {}
        self.velo_zaehler = -5
        self.velocity = 0.0
        self.velocity_in_coordinates = 0.0066  

    def init_bool_callback(self, msg):
        if msg.data:
            self.zero_position_set = True
            
            self.publish_conveyor_belt_marker() 

    def robot_position_callback(self, msg):
        if not self.zero_position_set:
            return

        if self.zero_position['x'] is None:
            self.zero_position['x'] = -msg.pos_x
            self.zero_position['y'] = msg.pos_y
            self.zero_position['z'] = msg.pos_z
            self.get_logger().info('Zero position has been set.')
            

        if None in self.zero_position.values():
            self.get_logger().warn('Zero position is not fully set.')
            return

        
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "robot"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = (-msg.pos_x - self.zero_position['x'])
        marker.pose.position.y = msg.pos_y - self.zero_position['y']
        marker.pose.position.z = -msg.pos_z - self.zero_position['z'] + 0.1
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05  
        marker.scale.y = 0.05  
        marker.scale.z = 0.05   
        marker.color.a = 1.0   
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self.robot_marker_pub.publish(marker)

        
        for object_id in self.objects.keys():
            self.update_object_position(object_id)

    def object_data_callback(self, msg):
        if not self.zero_position_set:
            return

        if None in self.zero_position.values():
            self.get_logger().warn('Zero position is not fully set.')
            return

        object_id = msg.index_value

        if object_id not in self.objects:
            self.objects[object_id] = {
                'x': (-0.00009823 * msg.object_pos_x) - (0.00009554 * (1017 - msg.object_pos_y)) + 0.27329,
                'y': (0.00000110 * msg.object_pos_x) - (0.00000725 * (1017 - msg.object_pos_y)) + 0.04866,
                'timestamp': time.time(),
                'class': msg.object_class
            }
            self.publish_object_marker(object_id)
        else:
            self.update_object_position(object_id)

    def publish_object_marker(self, object_id):
        obj = self.objects[object_id]
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "objects"
        marker.id = object_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = obj['x'] - 0.2
        marker.pose.position.y = obj['y'] 
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.08  
        marker.scale.y = 0.08  
        marker.scale.z = 0.003  
        if obj['class'] == 'unicorn':
            marker.color.a = 1.0   
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 1.0
        if obj['class'] == 'cat':
            marker.color.a = 1.0   
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        else:
            marker.color.a = 1.0   
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
        self.object_marker_pub.publish(marker)

    def update_object_position(self, object_id):
        current_time = time.time()
        obj = self.objects[object_id]
        time_diff = current_time - obj['timestamp']
        obj['x'] += self.velocity * time_diff
        obj['timestamp'] = current_time
        self.publish_object_marker(object_id)

    def publish_velocity_marker(self, velocity):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "velocity"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = 0.05  # Update this to the position where you want the arrow to start
        marker.pose.position.y = 0.3  # Update this to the position where you want the arrow to start
        marker.pose.position.z = 0.02  # Update this to the position where you want the arrow to start
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 10 * velocity  # Length of the arrow
        marker.scale.y = 0.02  # Width of the arrow shaft
        marker.scale.z = 0.02  # Width of the arrow head
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.text = f"Velocity: {velocity:.2f}"
        self.velocity_marker_pub.publish(marker)

    def publish_conveyor_belt_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "conveyor_belt"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.05
        marker.pose.position.z = -0.01  # Slightly below the object
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0  # Length of the conveyor belt
        marker.scale.y = 0.2  # Width of the conveyor belt
        marker.scale.z = 0.01  # Height of the conveyor belt
        marker.color.a = 1.0
        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 0.5
        self.conveyor_belt_marker_pub.publish(marker)

    def velocity_callback(self, msg):
        if self.velo_zaehler == 0:
            self.velocity = msg.data * self.velocity_in_coordinates
        if self.velo_zaehler > 1:
            self.velocity = (self.velocity * self.velo_zaehler + msg.data * self.velocity_in_coordinates) / (self.velo_zaehler + 1)
            self.get_logger().info(f'Updated velocity: {self.velocity}')
        self.velo_zaehler += 1
        self.publish_velocity_marker(self.velocity)

def main(args=None):
    rclpy.init(args=args)
    node = RVizPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
