#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import State, TransitionCallbackReturn, Node

class LidarLifecycleNode(Node):
    def __init__(self, node_name,**kwargs):
        super().__init__(node_name, **kwargs)
        self.get_logger().info(f"Initializing LiDAR node '{self.get_name()}'")
        self.lidar_sensor = None # Placeholder for the lidar sensor object
        self.is_configured = False

    def on_configure(self, state:State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Configuring LiDAR for node '{self.get_name()}'")
        try:
            self.lidar_sensor = None # Placeholder for the directional microphone object to be initialized
            self.lidar_sensor.initialize()
            self.lidar_sensor.set_parameters() # Placeholder for setting up the parameters for the lidar sensor
            #set up lidar parameters or whatever is needed
            self.is_configured = True
            self.get_logger().info("LiDAR successfully configured.")
            return TransitionCallbackReturn.SUCCESS
        
        except Exception as e:
            self.get_logger().error(f"Failed to configure LiDAR: {str(e)}")
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state:State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Activating LiDAR for node '{self.get_name()}'")
        if self.is_configured:
            self.lidar_sensor.start_scan() # Placeholder for starting the lidar scan
            self.get_logger().info("LiDAR scan started.")
            # Add more logic here for analyzing the LiDAR scan data and sending results to motion planning tasks
            return TransitionCallbackReturn.SUCCESS
        else:
            return TransitionCallbackReturn.FAILURE
    
    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Cleaning up LiDAR for node '{self.get_name()}'")
        self.destroy_mic(self.lidar_sensor) # Placeholder for cleaning up the directional microphone
        self.is_configured = False
        return TransitionCallbackReturn.SUCCESS
    
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Deactivating LiDAR for node '{self.get_name()}'")
        self.lidar_sensor.stop_scan() # Placeholder for stopping the lidar scan
        self.get_logger().info("LiDAR stream stopped.")
        # Add more logic here for ensuring that the LiDAR scan is stopped and motion planning tasks subsequently paused
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Shutting down LiDAR for node '{self.get_name()}'")
        self.lidar_sensor.destroy() # Placeholder for cleaning up the lidar sensor
        return TransitionCallbackReturn.SUCCESS


def main(args=None) -> None:
    rclpy.init(args=args)
    lidar_node = LidarLifecycleNode("lidar_node")
    rclpy.spin(lidar_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()  

    
