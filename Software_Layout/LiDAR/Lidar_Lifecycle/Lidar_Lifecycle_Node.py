#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import State, TransitionCallbackReturn, Node

class LidarLifecycleNode(Node):
    def __init__(self, node_name,**kwargs):
        super().__init__(node_name, **kwargs)
        self.get_logger().info(f"Initializing LiDAR Node '{self.get_name()}'")
        #Create hardware and software components
        

    def on_configure(self, state:State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Configuring LiDAR for node '{self.get_name()}'")
        try:
            #Initialize hardware and software components
            #set up LiDAR parameters or whatever is needed
            self.is_configured = True
            self.get_logger().info("LiDAR lifecycle successfully configured.")
            return TransitionCallbackReturn.SUCCESS
        
        except Exception as e:
            self.get_logger().error(f"Failed to configure LiDAR lifecycle: {str(e)}")
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state:State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Activating LiDAR lifecycle for node '{self.get_name()}'")
        #Start the LiDAR stream and processing here
        if self.is_configured:
            #Start LiDAR workflow
            self.get_logger().info("LiDAR stream started.")
            #Add logic calls here for processing the LiDAR stream and forwarding results to Microcontroller then Motor Controller
            return TransitionCallbackReturn.SUCCESS
        else:
            #Error handling if not configured
            return TransitionCallbackReturn.FAILURE
    
    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Cleaning up LiDAR lifecycle for node '{self.get_name()}'")
        #Add logic here to clean up any resources used for LiDAR
        self.is_configured = False
        return TransitionCallbackReturn.SUCCESS
    
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Deactivating LiDAR lifecycle for node '{self.get_name()}'")
        #Stop LiDAR stream if active
        self.get_logger().info("LiDAR stopped.")
        #Add more logic here for ensuring that the LiDAR stream is stopped
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Shutting down LiDAR lifecycle for node '{self.get_name()}'")
        #Add logic here to clean up any resources or data structures used for LiDAR --> destroy objects to free memory
        return TransitionCallbackReturn.SUCCESS


def main(args=None) -> None:
    rclpy.init(args=args)
    Lidar_lifecycle_node = LidarLifecycleNode("Lidar_lifecycle_node")
    rclpy.spin(Lidar_lifecycle_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()  

    
