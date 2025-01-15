import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import State, Transition
import asyncio

class LifecycleManager(Node):
    def __init__(self):
        super().__init__('lifecycle_manager')
        # List of subsystems
        self.subsystems = ['motor drive', 'lidar', 'audio', 'io', 'ultrasonics']
        # Dynamically create clients for each subsystem
        self.change_state_clients = {
            subsystem: self.create_client(ChangeState, f'/{subsystem}/change_state')
            for subsystem in self.subsystems
        }
        self.get_state_clients = {
            subsystem: self.create_client(GetState, f'/{subsystem}/get_state')
            for subsystem in self.subsystems
        }
        self.timer_period = 2.0  # 2 seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.current_state = 0  # 0 = Unconfigured, 1 = Active, 2 = Idle, 3 = Error
        self.starting_coordinate = (0, 0)  # Figure out starting coordinate data type

    async def get_subsystem_state(self, subsystem):
        # Asynchronously get the current state of a subsystem
        state_request = self.get_state_clients[subsystem].call_async(GetState.Request())
        state_response = await state_request
        return state_response.current_state

    def timer_callback(self):
        self.get_logger().info('Checking subsystem statuses...')
        tasks = [self.get_subsystem_state(subsystem) for subsystem in self.subsystems]
        # Run all state checks concurrently
        states = asyncio.gather(*tasks)
        states_result = self.get_node().get_executor().spin_until_future_complete(states)
        
        # Check all states and handle errors
        for subsystem, state in zip(self.subsystems, states_result):
            self.get_logger().info(f"{subsystem} current state: {state}")
            if state == State.FAILED:
                self.get_logger().info(f"Error detected in {subsystem}, transitioning all subsystems to inactive.")
                self.transition_all_to_inactive()
                break

    def transition_all_to_inactive(self):
        for subsystem in self.subsystems:
            self.get_logger().info(f"Transitioning {subsystem} to inactive...")
            change_state_request = ChangeState.Request(transition=Transition.TRANSITION_DEACTIVATE)
            self.change_state_clients[subsystem].call_async(change_state_request)
            self.get_logger().info(f"{subsystem} transitioned to inactive.")

    def transition_all_to_active(self):
        for subsystem in self.subsystems:
            self.get_logger().info(f"Transitioning {subsystem} to active...")
            change_state_request = ChangeState.Request(transition=Transition.TRANSITION_ACTIVATE)
            self.change_state_clients[subsystem].call_async(change_state_request)
            self.get_logger().info(f"{subsystem} transitioned to active.")

    def shutdown_all(self):
        for subsystem in self.subsystems:
            self.get_logger().info(f"Shutting down {subsystem}...")
            change_state_request = ChangeState.Request(transition=Transition.TRANSITION_CLEANUP)
            self.change_state_clients[subsystem].call_async(change_state_request)
            self.get_logger().info(f"{subsystem} shut down.")

    def configure_all(self):
        for subsystem in self.subsystems:
            self.get_logger().info(f"Configuring {subsystem}...")
            change_state_request = ChangeState.Request(transition=Transition.TRANSITION_CONFIGURE)
            self.change_state_clients[subsystem].call_async(change_state_request)
            self.get_logger().info(f"{subsystem} configured.")


def main(args=None):
    rclpy.init(args=args)
    lifecycle_manager = LifecycleManager()
    
    # Spin the node in an event loop
    rclpy.spin(lifecycle_manager)
    lifecycle_manager.transition_all_to_inactive()

    lifecycle_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
