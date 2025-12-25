import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistConverter(Node):
    def __init__(self):
        super().__init__('twist_converter')
        
        # Luister naar Nav2 (Zonder stempel)
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)
            
        # Spreek tegen de Robot (Met stempel)
        self.publisher = self.create_publisher(
            TwistStamped,
            '/cmd_vel_stamped',
            10)
            
        self.get_logger().info('Vertaler gestart: Twist -> TwistStamped')

    def listener_callback(self, msg):
        # Maak een nieuw bericht MET stempel
        new_msg = TwistStamped()
        new_msg.header.stamp = self.get_clock().now().to_msg()
        new_msg.header.frame_id = 'base_link'
        
        # Kopieer de snelheden
        new_msg.twist = msg
        
        # Stuur door
        self.publisher.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    converter = TwistConverter()
    rclpy.spin(converter)
    converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()