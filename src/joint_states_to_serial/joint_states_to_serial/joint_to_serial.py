import rclpy
from rclpy.node import Node
from moveit_msgs.msg import MotionPlanRequest
from math import degrees
import serial

class MotionPlanSubscriber(Node):
    def __init__(self):
        super().__init__('motion_plan_subscriber')
        
        # Initialize serial communication
        try:
            self.serial_port = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1)
            self.get_logger().info("Serial port /dev/ttyUSB0 opened successfully")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.serial_port = None
        
        # Subscribe to motion_plan_request topic
        self.subscription = self.create_subscription(
            MotionPlanRequest,
            'motion_plan_request',
            self.motion_plan_callback,
            10
        )
        self.get_logger().info("Subscribed to motion_plan_request topic")

    def motion_plan_callback(self, msg):
        if not msg.goal_constraints:
            self.get_logger().info("No goal constraints found")
            return

        joint_positions = {jc.joint_name: degrees(jc.position) for constraint in msg.goal_constraints for jc in constraint.joint_constraints}
        
        # Custom joint order with joint1 set to 0
        custom_order = ['joint3', 'joint2', 'joint4', 'joint6', 'joint5', 'joint1']
        joint_angles_in_order = [joint_positions.get(joint, 0.0) for joint in custom_order]
        joint_angles_in_order[-1] = 0.0  # Force joint1 angle to 0
        
        # Format the message
        joint_angles_str = ' '.join(f"{angle:.2f}" for angle in joint_angles_in_order)
        message = f"M{joint_angles_str}\n"
        self.get_logger().info(f"Prepared message: {message.strip()}")
        
        # Send the message via serial port
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(message.encode())
                self.get_logger().info("Message sent over serial")
            except serial.SerialException as e:
                self.get_logger().error(f"Failed to send message: {e}")
        else:
            self.get_logger().error("Serial port is not open")

    def destroy_node(self):
        # Close the serial port on shutdown
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info("Serial port closed")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotionPlanSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
