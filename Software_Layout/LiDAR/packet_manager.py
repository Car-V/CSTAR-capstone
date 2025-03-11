import rclpy
from rclpy.node import Node
from sensor_msgs.msg import AudioData  # Replace with your actual message type
from nav_msgs.msg import Odometry  # Replace with the actual type of positional data
import websockets
import asyncio
import json
import time

class PacketManager(Node):
    def __init__(self):
        super().__init__('packet_manager')
        self.audio_data = None
        self.positional_data = None

        # Buffers to hold incoming data
        self.audio_buffer = []
        self.positional_buffer = []

        # Set batch interval (e.g., 100ms)
        self.batch_interval = 0.1  # in seconds (100ms)
        self.last_batch_time = time.time()

        # Create subscribers for both audio and positional data
        self.audio_subscription = self.create_subscription(
            AudioData,  # Replace with the actual message type
            '/audio_topic',  # Replace with the correct audio topic
            self.audio_callback,
            10
        )

        self.positional_subscription = self.create_subscription(
            Odometry,  # Replace with the actual message type from slam_toolbox
            '/slam/pose',  # Replace with the actual positional topic
            self.positional_callback,
            10
        )

        # WebSocket URL for the host PC
        self.websocket_url = "ws://your_host_pc_address:port"
        self.websocket = None

    def audio_callback(self, msg):
        """Callback to handle incoming audio data"""
        self.audio_data = msg  # Store incoming audio data
        self.audio_buffer.append(self.audio_data)
        self.get_logger().info('Received audio data')

    def positional_callback(self, msg):
        """Callback to handle incoming positional data"""
        self.positional_data = msg  # Store incoming positional data
        self.positional_buffer.append(self.positional_data)
        self.get_logger().info('Received positional data')

    def try_publish_packet(self):
        """Check if it's time to send a batch of packets"""
        current_time = time.time()
        
        # Check if it's time to send a batch of data
        if current_time - self.last_batch_time >= self.batch_interval:
            # Prepare a batch of packets
            batch = []
            
            # Pair audio and positional data based on time proximity
            while self.audio_buffer and self.positional_buffer:
                audio_msg = self.audio_buffer.pop(0)
                pos_msg = self.positional_buffer.pop(0)

                # Match data based on timestamps (allow for some delay)
                if abs(audio_msg.header.stamp.sec - pos_msg.header.stamp.sec) < 0.1 and \
                   abs(audio_msg.header.stamp.nanosec - pos_msg.header.stamp.nanosec) < 100000000:  # Threshold 100ms
                    packet = {
                        "audio_data": audio_msg.data,  # Adjust based on your audio message type
                        "positional_data": {
                            "x": pos_msg.pose.pose.position.x,
                            "y": pos_msg.pose.pose.position.y,
                            "z": pos_msg.pose.pose.position.z
                        },
                        "timestamp": time.time()  # Timestamp for packet creation
                    }
                    batch.append(packet)

            if batch:
                # Send the batch over WebSocket
                asyncio.run(self.send_packet(batch))

            # Update last batch time
            self.last_batch_time = current_time

    async def send_packet(self, batch):
        """Send the batch of packets to the host PC over WebSocket"""
        if self.websocket is None:
            self.websocket = await websockets.connect(self.websocket_url)
        
        # Send the batch as a JSON array
        await self.websocket.send(json.dumps(batch))
        self.get_logger().info(f"Sent batch of {len(batch)} packets")

def main(args=None):
    rclpy.init(args=args)
    packet_manager = PacketManager()
    
    # Use a timer to periodically check if it's time to send the batch
    packet_manager.create_timer(packet_manager.batch_interval, packet_manager.try_publish_packet)
    
    rclpy.spin(packet_manager)
    packet_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
