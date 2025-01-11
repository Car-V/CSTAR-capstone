#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import State, TransitionCallbackReturn, Node

class LifecycleNode(Node):
    def __init__(self, node_name,**kwargs):
        super().__init__(node_name, **kwargs)
        self.get_logger().info("LifecycleNode: Initializing")

    def on_configure(self, state:State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Node '{self.get_name()}' is in state '{state.label}'. Transitioning to 'configure'")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state:State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Node '{self.get_name()}' is in state '{state.label}'. Transitioning to 'activate'")
        return TransitionCallbackReturn.SUCCESS
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Node '{self.get_name()}' is in state '{state.label}'. Transitioning to 'deactivate'")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Node '{self.get_name()}' is in state '{state.label}'. Transitioning to 'shutdown'")
        return TransitionCallbackReturn.SUCCESS


def main(args=None) -> None:
    rclpy.init(args=args)
    lifecycle_node = LifecycleNode("lifecycle_node")
    rclpy.spin(lifecycle_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()  

    
