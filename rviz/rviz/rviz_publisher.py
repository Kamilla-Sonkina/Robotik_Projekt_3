import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from ro45_portalrobot_interfaces.msg import RobotPos, RobotCmd
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
        self.robot_command_sub = self.create_subscription(RobotCmd, 'robot_command', self.command_callback, 10)
        self.box_marker_pub = self.create_publisher(Marker, 'rviz_box_marker', 10)
        self.accel_marker_pub = self.create_publisher(Marker, 'rviz_accel_marker', 10)
        self.zero_position = {'x': None, 'y': None, 'z': None}
        self.zero_position_set = False
        self.objects = {}
        self.velo_zaehler = -5
        self.velocity = 0.0
        self.velocity_in_coordinates = 0.0066
        self.u_x = 0
        self.u_y = 0
        self.u_z = 0
        self.last_object_timestamp = 0
        self.robot_pos = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.activate_gripper = False
        self.gripped_object_id = None
        self.box_cat = {'x': 0.0854, 'y': 0.001, 'z': 0.0562} 
        self.box_unicorn = {'x': 0.0012, 'y': 0.001, 'z': 0.0562}
        self.weight_measure = 0.8
        self.weight_velocity_sub = 0.2
        self.last_object_pose = 0
        self.object_velocity = 0

    def init_bool_callback(self, msg):
        if msg.data:
            self.zero_position_set = True
            self.publish_conveyor_belt_marker()
            
            
    def command_callback(self, msg):
        self.u_x = msg.accel_x
        self.u_y = msg.accel_y
        self.u_z = msg.accel_z
        self.activate_gripper = msg.activate_gripper
        self.publish_accel_markers()

    def robot_position_callback(self, msg):
        
        if not self.zero_position_set:
            return

        if self.zero_position['x'] is None:
            self.zero_position['x'] = -msg.pos_x
            self.zero_position['y'] = msg.pos_y
            self.zero_position['z'] = msg.pos_z
            self.get_logger().info('Zero position has been set.')
            self.publish_boxes()


        if None in self.zero_position.values():
            self.get_logger().warn('Zero position is not fully set.')
            return

        self.robot_pos['x'] = -msg.pos_x - self.zero_position['x']
        self.robot_pos['y'] = msg.pos_y - self.zero_position['y']
        self.robot_pos['z'] = msg.pos_z - self.zero_position['z']

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "robot"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = -msg.pos_x - self.zero_position['x']
        marker.pose.position.y = msg.pos_y - self.zero_position['y']
        marker.pose.position.z = msg.pos_z - self.zero_position['z'] - 0.025
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 0.5
        self.robot_marker_pub.publish(marker)

        if self.activate_gripper and self.gripped_object_id is not None and self.gripped_object_id !=0:
            self.update_gripped_object_position(self.gripped_object_id)

        for object_id in list(self.objects.keys()):
            self.update_object_position(object_id, msg, False)

    def object_data_callback(self, msg):
        if not self.zero_position_set:
            return

        if None in self.zero_position.values():
            self.get_logger().warn('Zero position is not fully set.')
            return

        object_id = msg.index_value

        if object_id not in self.objects:
            self.objects[object_id] = {
                'x': ((-0.000174 * msg.object_pos_x) + 0.179 ),
                'y': (-0.000223*msg.object_pos_y + 0.09 ),
                'z': 0.079,
                'timestamp': msg.timestamp_value,
                'class': msg.object_class
            }
            self.publish_object_marker(object_id)
        else:
            self.update_object_position(object_id, msg, True)

    def publish_object_marker(self, object_id):
        obj = self.objects[object_id]
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "objects"
        marker.id = object_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = obj['x'] 
        marker.pose.position.y = obj['y']
        marker.pose.position.z = obj['z']
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.035
        marker.scale.y = 0.03
        marker.scale.z = 0.003
        marker.color.a = 1.0
        if obj['class'] == 'unicorn':
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        elif obj['class'] == 'cat':
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
        else:
            marker.color.r = 0.3
            marker.color.g = 0.3
            marker.color.b = 0.3
        self.object_marker_pub.publish(marker)

    def update_object_position(self, object_id, msg, new_object):
        self.get_logger().debug(f'in update object id:{object_id}')
        if(self.activate_gripper):
            obj = self.objects[object_id]
            if((abs(obj['x'] - self.robot_pos['x']) < 0.035) 
               and (abs(obj['y'] - self.robot_pos['y']) < 0.005)):
                self.gripped_object_id = object_id
        if object_id == self.gripped_object_id:
            self.update_gripped_object_position(object_id)
        elif not self.activate_gripper and object_id == self.gripped_object_id:
            self.delete_object_marker(object_id)
            del self.objects[object_id]
            self.gripped_object_id = None
        elif new_object:
            current_time = time.time()
            obj = self.objects[object_id]
            time_diff = current_time - obj['timestamp']
            self.last_object_pose = obj['x']
            self.last_object_timestamp = obj['timestamp']
            obj['timestamp'] = msg.timestamp_value
            obj['x'] = (-0.000174 * msg.object_pos_x) + 0.179 
            self.object_velocity = (obj['x'] - self.last_object_pose) / (obj['timestamp'] - self.last_object_timestamp)
            obj['x'] -= self.velocity * time_diff
            if obj['x'] < 0:
                self.delete_object_marker(object_id)
                del self.objects[object_id]
                self.get_logger().info(f'Object {object_id} has been removed from the scene.')
                return 
            obj['timestamp'] = current_time
            self.publish_object_marker(object_id)
            self.get_logger().debug(f'Object {object_id} has been updated from new input.')
        else:
            current_time = time.time()
            obj = self.objects[object_id]
            time_diff = current_time - obj['timestamp']
            obj['x'] -= self.velocity * time_diff
            if obj['x'] < 0:
                self.delete_object_marker(object_id)
                del self.objects[object_id]
                self.get_logger().info(f'Object {object_id} has been removed from the scene.')
                return 
            obj['timestamp'] = current_time
            self.publish_object_marker(object_id)
            self.get_logger().debug(f'Object {object_id} has been updated from old input.')

    def delete_object_marker(self, object_id):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "objects"
        marker.id = object_id
        marker.action = Marker.DELETE
        self.object_marker_pub.publish(marker)

    def update_gripped_object_position(self, object_id):
        if(self.activate_gripper == False):
            self.delete_object_marker(object_id)
            del self.objects[object_id]
        else:
            obj = self.objects[self.gripped_object_id]
            obj['x'] = self.robot_pos['x'] 
            obj['y'] = self.robot_pos['y'] 
            obj['z'] = self.robot_pos['z']
            obj['timestamp'] = time.time()
            self.publish_object_marker(self.gripped_object_id)

    def publish_velocity_marker(self, velocity):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "velocity"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = 0.05
        marker.pose.position.y = 0.1
        marker.pose.position.z = 0.04
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = -10 * velocity
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.text = f"Velocity: {velocity:.2f}"
        self.velocity_marker_pub.publish(marker)

    def publish_accel_markers(self):
        self.publish_accel_marker(self.u_x, "accel_x", 1.0, 0.0, 0.0, 1.0, 1, 0.0, 0.0, 0.0, 1.0)
        self.publish_accel_marker(self.u_y, "accel_y", 0.0, 1.0, 0.0, 1.0, 2, 0.0, 0.0, 0.7071, 0.7071)
        self.publish_accel_marker(self.u_z, "accel_z", 0.0, 0.0, 1.0, 1.0, 3, 0.0, -0.7071, 0.0, 0.7071)

    def publish_accel_marker(self, accel, ns, r, g, b, a, marker_id, o_x, o_y, o_z, o_w):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = self.robot_pos['x']
        marker.pose.position.y = self.robot_pos['y']
        marker.pose.position.z = self.robot_pos['z'] 

        
        marker.pose.orientation.x = o_x
        marker.pose.orientation.y = o_y
        marker.pose.orientation.z = o_z
        marker.pose.orientation.w = o_w
        marker.scale.x = accel
        marker.scale.y = 0.2 * accel
        marker.scale.z = 0.2 * accel
        

        marker.color.a = a
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        self.accel_marker_pub.publish(marker)


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
        marker.pose.position.z = 0.085
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 0.06
        marker.scale.z = 0.01
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.conveyor_belt_marker_pub.publish(marker)

    def velocity_callback(self, msg):
        if self.velo_zaehler == 0:
            self.velocity = msg.data * self.velocity_in_coordinates
        if self.velo_zaehler > 1:
            self.velocity = (self.velocity * self.velo_zaehler + msg.data * self.velocity_in_coordinates) / (self.velo_zaehler + 1)
            self.velocity = (self.velocity * self.velo_zaehler + msg.data * self.velocity_in_coordinates) / (self.velo_zaehler + 1)
            self.velocity = (self.velocity * self.weight_velocity_sub + self.object_velocity * self.weight_measure) / (self.weight_measure + self.weight_velocity_sub)
            self.get_logger().info(f'Updated velocity: {self.velocity}')
        self.velo_zaehler += 1
        self.publish_velocity_marker(self.velocity)
    
    def publish_boxes(self):
        self.publish_box_marker(self.box_cat, "box_cat", 1.0, 0.0, 0.0, 1.0)  
        self.publish_box_marker(self.box_unicorn, "box_unicorn", 1.0, 1.0, 0.0, 0.0)  
        
    
    def publish_box_marker(self, box, ns, a, r, g, b):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = box['x'] 
        marker.pose.position.y = box['y'] - 0.025
        marker.pose.position.z = box['z'] + 0.04
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.066
        marker.scale.y = 0.1
        marker.scale.z = 0.05
        marker.color.a = a
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        self.box_marker_pub.publish(marker)
        self.get_logger().info(f'published : {box}')


def main(args=None):
    rclpy.init(args=args)
    node = RVizPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
