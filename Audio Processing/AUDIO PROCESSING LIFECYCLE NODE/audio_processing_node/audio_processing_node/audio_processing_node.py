#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import State, TransitionCallbackReturn, Node

class AudioProcessingLifecycleNode(Node):
    def __init__(self, node_name,**kwargs):
        super().__init__(node_name, **kwargs)
        self.get_logger().info(f"Initializing Audio Processing Node '{self.get_name()}'")
        self.directional_microphone = None # Placeholder for the directional microphone object
        self.omnidirectional_microphone = None # Placeholder for the omnidirectional microphone object
        self.is_configured = False

    def on_configure(self, state:State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Configuring Audio Processing for node '{self.get_name()}'")
        try:
            # Initialize mircophones
            self.directional_microphone = None # Plceholder for the directional microphone object to be initialized
            self.directional_microphone.initialize()
            self.omnidirectional_microphone = None # Placeholder for the omnidirectional microphone object to be initialized
            self.omnidirectional_microphone.initialize()
            #set up audio processing parameters or whatever is needed
            self.is_configured = True
            self.get_logger().info("Audio Processing successfully configured.")
            return TransitionCallbackReturn.SUCCESS
        
        except Exception as e:
            self.get_logger().error(f"Failed to configure Audio Processing: {str(e)}")
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state:State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Activating Audio Processing for node '{self.get_name()}'")
        # Start the audio stream and processing here
        if self.is_configured:
            self.microphone.start_stream() # Placeholder for starting the audio stream
            self.get_logger().info("Audio stream started.")
            # Add more logic here for processing the audio stream and caching the results to be sent to PC
            return TransitionCallbackReturn.SUCCESS
        else:
            return TransitionCallbackReturn.FAILURE
    
    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Cleaning up Audio Processing for node '{self.get_name()}'")
        # Add logic here to clean up any resources or data structures used for audio processing
        self.self.directional_microphone.destroy() # Placeholder for cleaning up the directional microphone
        self.self.omnidirectional_microphone.destroy() # Placeholder for cleaning up the omnidirectional microphone
        self.is_configured = False
        return TransitionCallbackReturn.SUCCESS
    
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Deactivating Audio Processing for node '{self.get_name()}'")
        # Stop audio stream if active
        self.microphone.stop_stream() # Placeholder for stopping the audio stream
        self.get_logger().info("Audio stream stopped.")
        # Add more logic here for ensuring that the audio stream is stopped and samples are cached and forwarded to PC
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Shutting down Audio Processing for node '{self.get_name()}'")
        self.self.directional_microphone.destroy() # Placeholder for cleaning up the directional microphone
        self.self.omnidirectional_microphone.destroy() # Placeholder for cleaning up the omnidirectional microphone
        return TransitionCallbackReturn.SUCCESS


def main(args=None) -> None:
    rclpy.init(args=args)
    audio_processing_node = AudioProcessingLifecycleNode("audio_processing_node")
    rclpy.spin(audio_processing_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()  

    
