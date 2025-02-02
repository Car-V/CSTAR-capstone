import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import threading

class LiDARReader(Node):

    def __init__(self):
        super().__init__('lidar_reader')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.file = open('lidar_data.txt', 'w')

        # Start a timer to stop after 10 seconds
        self.timer = threading.Timer(10, self.stop_node)
        self.timer.start()

    def lidar_callback(self, msg):
        distances = msg.ranges
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        timestamp_sec = msg.header.stamp.sec
        timestamp_nanosec = msg.header.stamp.nanosec

        for i, distance in enumerate(distances):
            angle = angle_min + i * angle_increment
            data_line = f'Timestamp: {timestamp_sec}.{timestamp_nanosec}, Angle: {angle:.2f} rad, Distance: {distance:.2f} m\n'
            self.get_logger().info(data_line.strip())
            self.file.write(data_line)

    def stop_node(self):
        self.get_logger().info('Stopping LiDAR after 10 seconds')
        self.file.close()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    lidar_reader = LiDARReader()
    rclpy.spin(lidar_reader)
    lidar_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()