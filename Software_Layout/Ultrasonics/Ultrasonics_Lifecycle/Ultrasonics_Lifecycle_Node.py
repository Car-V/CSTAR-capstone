#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import State, TransitionCallbackReturn, Node

class UltrasonicLifecycleNode(Node):
    def __init__(self, node_name,**kwargs):
        super().__init__(node_name, **kwargs)
        self.get_logger().info(f"Initializing Ultrasonic Node '{self.get_name()}'")
        #Create hardware and software components
        

    def on_configure(self, state:State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Configuring Ultrasonic for node '{self.get_name()}'")
        try:
            #Initialize hardware and software components
            #set up Ultrasonic parameters or whatever is needed
            self.is_configured = True
            self.get_logger().info("Ultrasonic lifecycle successfully configured.")
            return TransitionCallbackReturn.SUCCESS
        
        except Exception as e:
            self.get_logger().error(f"Failed to configure Ultrasonic lifecycle: {str(e)}")
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state:State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Activating Ultrasonic lifecycle for node '{self.get_name()}'")
        #Start the Ultrasonic data flow and processing
        if self.is_configured:
            #Start Ultrasonic workflow
            self.get_logger().info("Ultrasonic stream started.")
            #Add logic here to forward Ultrasonic data results to Motor Control or other subsystems as needed
            return TransitionCallbackReturn.SUCCESS
        else:
            #Error handling if not configured
            return TransitionCallbackReturn.FAILURE
    
    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Cleaning up Ultrasonic lifecycle for node '{self.get_name()}'")
        #Add logic here to clean up any resources used for Ultrasonic
        self.is_configured = False
        return TransitionCallbackReturn.SUCCESS
    
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Deactivating Ultrasonic lifecycle for node '{self.get_name()}'")
        #Stop Ultrasonic stream if active
        self.get_logger().info("Ultrasonic stopped.")
        #Add more logic here for ensuring that the Ultrasonics are properly deactivated
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Shutting down Ultrasonic lifecycle for node '{self.get_name()}'")
        #Add logic here to clean up any resources or data structures used for Ultrasonic --> destroy objects to free memory
        return TransitionCallbackReturn.SUCCESS


def main(args=None) -> None:
    rclpy.init(args=args)
    Ultrasonic_lifecycle_node = UltrasonicLifecycleNode("Ultrasonic_lifecycle_node")
    rclpy.spin(Ultrasonic_lifecycle_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()  

    
