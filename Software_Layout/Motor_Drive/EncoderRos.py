import math
import time
import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node
# from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Odometry
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion

class Encoder:
    WHEEL_DIAMETER = 0.072  
    PULSES_PER_REV = 537.7  
    GEAR_RATIO = 19.2       
    WHEEL_BASE = 0.25 

    def __init__(self, pin_a: int, pin_b: int):
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.position = 0
        self.direction = 0  # 1 for clockwise, -1 for counterclockwise
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin_a, GPIO.IN)
        GPIO.setup(self.pin_b, GPIO.IN)
        
        self.last_a_state = GPIO.input(self.pin_a)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = time.time()

    def update_position(self):
        # Read the state of the encoder channel A
        current_a_state = GPIO.input(self.pin_a)
        current_b_state = GPIO.input(self.pin_b)

        # Check if the A channel changed
        if current_a_state != self.last_a_state:
            # If the A channel leads the B channel, it's a clockwise rotation
            if current_b_state != current_a_state:
                self.position += 1
                self.direction = 1  # Clockwise
            else:
                self.position -= 1
                self.direction = -1  # Counterclockwise

        self.last_a_state = current_a_state  # Update the last state for next comparison

    def get_position(self):
        return self.position

    def get_direction(self):
        return self.direction

    def reset_position(self):
        self.position = 0

    def calculate_distance(self):
        wheel_circumference = math.pi * self.WHEEL_DIAMETER
        distance_per_pulse = wheel_circumference / (self.PULSES_PER_REV * self.GEAR_RATIO)
        return self.position * distance_per_pulse

    def update_odometry(self, left_encoder, right_encoder):
        left_distance = left_encoder.calculate_distance()
        right_distance = right_encoder.calculate_distance()
        
        distance = (right_distance + left_distance) / 2.0
        delta_theta = (right_distance - left_distance) / self.WHEEL_BASE
        
        self.theta += delta_theta
        self.x += distance * math.cos(self.theta)
        self.y += distance * math.sin(self.theta)

    def get_odometry(self):
        return self.x, self.y, self.theta


class EncoderOdometryNode(Node):
    def __init__(self):
        super().__init__('encoder_odometry_node')

        self.left_encoder = Encoder(pin_a=17, pin_b=18)  # NEED TO UPDATE PIN
        self.right_encoder = Encoder(pin_a=22, pin_b=23)  # NEED TO UPDATE PIN

        #self.odometry_publisher = self.create_publisher(Float32MultiArray, 'odometry', 10)
        
        self.odometry_publisher = self.ceate_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

    def timer_callback(self):
        
        
        self.left_encoder.update_position()
        self.right_encoder.update_position()

        # Do we wat to save previous Odom values?  
        # Might need it to get velocity?
        
        left_distance = self.left_encoder.calculate_distance()
        
        self.left_encoder.update_odometry(self.left_encoder, self.right_encoder)
xx

        # odometry = Float32MultiArray()
        # odometry.data = [self.left_encoder.x, self.left_encoder.y, self.left_encoder.theta]
        
        x = self.left_encoder.x
        y = self.left_encoder.y
        theta = self.left_encoder.theta
        
        current_time = self.get_clock().now.to_msg()
        # Quaternion needed 
        
        # Potential Broadcast (odom -> baselink)
        
        # this is tf_message 
        
        transform = TransformedStamped()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        
        # this is broadcasting a new tf 
        
        transform.header.stamp = current_time
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = 0.0
        
        # ORIENTATION 
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 0.0
        
        self.tf_broadcaster.sendTransform(transform)
        
        # PUBLISH ODOMETRY OVER ROS
        
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.header.stamp = current_time
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
                
        # ORIENTATION
        # in the future when we calculate rotation we can have orientation = Quaternion(x,y,z,w)
        # then we can just do odom.pose.pose = orientation
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = 0.0
        odom.pose.pose.orientation.w = 0.0
        
        self.odometry_publisher.publish(odom)
    
        # self.odometry_publisher.publish(odometry)
        
        
        

def main(args=None):
    rclpy.init(args=args)

    encoder_odometry_node = EncoderOdometryNode()

    rclpy.spin(encoder_odometry_node)

    encoder_odometry_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
