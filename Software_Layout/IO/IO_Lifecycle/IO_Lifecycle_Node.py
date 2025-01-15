#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import State, TransitionCallbackReturn, Node

class IOLifecycleNode(Node):
    def __init__(self, node_name,**kwargs):
        super().__init__(node_name, **kwargs)
        self.get_logger().info(f"Initializing IO Node '{self.get_name()}'")
        #Create hardware and software components
        

    def on_configure(self, state:State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Configuring IO for node '{self.get_name()}'")
        try:
            #Initialize hardware and software components
            #set up IO parameters or whatever is needed
            self.is_configured = True
            self.get_logger().info("IO lifecycle successfully configured.")
            return TransitionCallbackReturn.SUCCESS
        
        except Exception as e:
            self.get_logger().error(f"Failed to configure IO lifecycle: {str(e)}")
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state:State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Activating IO lifecycle for node '{self.get_name()}'")
        #Start the IO data flow from lifecycle manager to Button Indicator LED
        if self.is_configured:
            #Start IO workflow
            self.get_logger().info("IO stream started.")
            #Add call to IO Indicator function to subscribe to lifecycle managers current state
            #and translate that to a button color
            return TransitionCallbackReturn.SUCCESS
        else:
            #Error handling if not configured
            return TransitionCallbackReturn.FAILURE
    
    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Cleaning up IO lifecycle for node '{self.get_name()}'")
        #Add logic here to clean up any resources used for IO
        self.is_configured = False
        return TransitionCallbackReturn.SUCCESS
    
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Deactivating IO lifecycle for node '{self.get_name()}'")
        #Stop IO stream if active
        self.get_logger().info("IO stopped.")
        #Add more logic here for ensuring that the IO is stopped and indicator LED is off
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Shutting down IO lifecycle for node '{self.get_name()}'")
        #Add logic here to clean up any resources or data structures used for IO --> destroy objects to free memory
        return TransitionCallbackReturn.SUCCESS


def main(args=None) -> None:
    rclpy.init(args=args)
    IO_lifecycle_node = IOLifecycleNode("IO_lifecycle_node")
    rclpy.spin(IO_lifecycle_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()  

    
