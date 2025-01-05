import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class ImageRePublisher(Node):
    def __init__(self):
        super().__init__('image_republisher')
        
        # Parameter for new frame_id
        self.declare_parameter('new_frame_id', 'camera_link_optical')
        self.new_frame_id = self.get_parameter('new_frame_id').get_parameter_value().string_value

        # Publisher for the modified Image
        self.publisher = self.create_publisher(Image, '/camera/image_fix', 10)
        
        # Subscriber to the original Image topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image',  # Original topic
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        # Modify the frame_id in the message header
        modified_msg = Image()
        modified_msg.header = msg.header
        modified_msg.header.frame_id = self.new_frame_id  # Replace frame_id
        modified_msg.height = msg.height
        modified_msg.width = msg.width
        modified_msg.encoding = msg.encoding
        modified_msg.is_bigendian = msg.is_bigendian
        modified_msg.step = msg.step
        modified_msg.data = msg.data

        # Publish the modified message
        self.publisher.publish(modified_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImageRePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()