#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import State, TransitionCallbackReturn, Node

class MotorDriveLifecycleNode(Node):
    def __init__(self, node_name,**kwargs):
        super().__init__(node_name, **kwargs)
        self.get_logger().info(f"Initializing Motor Drive Node '{self.get_name()}'")
        #Create hardware and software components
        

    def on_configure(self, state:State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Configuring Motor Drive for node '{self.get_name()}'")
        try:
            #Initialize hardware and software components
            #set up Motor Drive parameters or whatever is needed
            self.is_configured = True
            self.get_logger().info("Motor Drive lifecycle successfully configured.")
            return TransitionCallbackReturn.SUCCESS
        
        except Exception as e:
            self.get_logger().error(f"Failed to configure Motor Drive lifecycle: {str(e)}")
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state:State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Activating Motor Drive lifecycle for node '{self.get_name()}'")
        #Start Motor Drive
        if self.is_configured:
            #Start Motor Drive workflow
            self.get_logger().info("Motor Drive stream started.")
            #Add logic calls here for Motor Drive
            return TransitionCallbackReturn.SUCCESS
        else:
            #Error handling if not configured
            return TransitionCallbackReturn.FAILURE
    
    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Cleaning up Motor Drive lifecycle for node '{self.get_name()}'")
        #Add logic here to clean up any resources used for Motor Drive
        self.is_configured = False
        return TransitionCallbackReturn.SUCCESS
    
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Deactivating Motor Drive lifecycle for node '{self.get_name()}'")
        #Stop Motor Drive if active
        self.get_logger().info("Motor Drive stopped.")
        #Add more logic here for ensuring that Motor Drive is stopped
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Shutting down Motor Drive lifecycle for node '{self.get_name()}'")
        #Add logic here to clean up any resources or data structures used for Motor Drive --> destroy objects to free memory
        return TransitionCallbackReturn.SUCCESS


def main(args=None) -> None:
    rclpy.init(args=args)
    Motor_Drive_lifecycle_node = MotorDriveLifecycleNode("Motor_Drive_lifecycle_node")
    rclpy.spin(Motor_Drive_lifecycle_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()  

    
