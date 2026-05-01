import rclpy
from rclpy.node import Node
from nav_interfaces.msg import TargetGoal
import threading
import sys

class UserInterface(Node):
    def __init__(self):
        super().__init__('ui_node')
        # Create a publisher that will send the goal to the C++ Action Client
        self.publisher_ = self.create_publisher(TargetGoal, 'ui_target', 10)
        
        # Start keyboard input in a separate thread so it doesn't block ROS spin
        self.input_thread = threading.Thread(target=self.get_user_input)
        # Daemon thread means it will close automatically when the main program closes
        self.input_thread.daemon = True 
        self.input_thread.start()

    def get_user_input(self):
        while rclpy.ok():
            print("\n--- Robot Navigation UI ---")
            print("1. Set new target (x, y, theta)")
            print("2. Cancel current target")
            print("3. Exit")
            
            try:
                choice = input("Select an option: ")
            except EOFError:
                break

            msg = TargetGoal()
            
            if choice == '1':
                try:
                    msg.x = float(input("Enter target X: "))
                    msg.y = float(input("Enter target Y: "))
                    msg.theta = float(input("Enter target Theta: "))
                    msg.is_cancel = False
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"Target sent: x={msg.x}, y={msg.y}, theta={msg.theta}")
                except ValueError:
                    print("Invalid input. Please enter numbers only.")
                    
            elif choice == '2':
                msg.is_cancel = True
                self.publisher_.publish(msg)
                self.get_logger().warn("Cancel command sent to Action Client!")
                
            elif choice == '3':
                print("Exiting UI...")
                # We raise a SystemExit exception to cleanly shut down
                rclpy.shutdown()
                sys.exit(0)
            else:
                print("Invalid choice.")

def main(args=None):
    rclpy.init(args=args)
    node = UserInterface()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
