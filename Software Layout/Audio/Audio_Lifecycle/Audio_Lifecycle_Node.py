#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import State, TransitionCallbackReturn, Node

class AudioLifecycleNode(Node):
    def __init__(self, node_name,**kwargs):
        super().__init__(node_name, **kwargs)
        self.get_logger().info(f"Initializing Audio Node '{self.get_name()}'")
        #Create hardware and software components
        

    def on_configure(self, state:State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Configuring Audio for node '{self.get_name()}'")
        try:
            #Initialize hardware and software components
            #set up audio processing parameters or whatever is needed
            self.is_configured = True
            self.get_logger().info("Audio lifecycle successfully configured.")
            return TransitionCallbackReturn.SUCCESS
        
        except Exception as e:
            self.get_logger().error(f"Failed to configure Audio lifecycle: {str(e)}")
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state:State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Activating Audio lifecycle for node '{self.get_name()}'")
        # Start the audio stream and processing here
        if self.is_configured:
            #Start audio stream and processing or whatever the workflow is
            self.get_logger().info("Audio stream started.")
            #Add more logic here for processing the audio stream and caching the results to be sent to PC
            return TransitionCallbackReturn.SUCCESS
        else:
            #Error handling if the audio processing is not configured
            return TransitionCallbackReturn.FAILURE
    
    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Cleaning up Audio lifecycle for node '{self.get_name()}'")
        #Add logic here to clean up any resources or data structures used for audio processing
        self.is_configured = False
        return TransitionCallbackReturn.SUCCESS
    
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Deactivating Audio lifecycle for node '{self.get_name()}'")
        #Stop audio stream if active
        self.get_logger().info("Audio stream stopped.")
        #Add more logic here for ensuring that the audio stream is stopped and samples are cached and forwarded to PC
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Shutting down Audio lifecycle for node '{self.get_name()}'")
        #Add logic here to clean up any resources or data structures used for audio processing --> destroy objects to free memory
        return TransitionCallbackReturn.SUCCESS


def main(args=None) -> None:
    rclpy.init(args=args)
    audio_lifecycle_node = AudioLifecycleNode("audio_lifecycle_node")
    rclpy.spin(audio_lifecycle_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()  

    
