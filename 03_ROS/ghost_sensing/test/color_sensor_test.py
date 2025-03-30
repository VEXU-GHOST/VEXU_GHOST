# pip install pygame
# python3 03_ROS/ghost_sensing/test/color_sensor_test.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
import pygame

# Initialize pygame for window display
pygame.init()
screen = pygame.display.set_mode((400, 400))
pygame.display.set_caption("Color Display")

class ColorSubscriber(Node):
    def __init__(self):
        super().__init__('color_subscriber')
        # Create a subscriber for the ColorRGBA topic
        self.subscription = self.create_subscription(
            ColorRGBA,
            '/sensors/color_sensor_0/hsv',  # Replace with the actual topic name
            self.color_callback,
            10
        )
        self.color = pygame.Color(0, 0, 0)  # Default color (black)

    def color_callback(self, msg):
        # Extract RGBA values from the ColorRGBA message
        # Adjusting color values based on the new requirement
        h = msg.r
        s = msg.g   * 10000
        v = msg.b  * 20

        # Clamp the values to be between 0 and 255
        s = max(0, min(1, s))
        v = max(0, min(1, v))

        self.color.hsva = (int(h),int(s*100),int(v*100), 0)

        # Print the callback message and color values for debugging
        print('called back', msg, self.color)
def main(args=None):
    rclpy.init(args=args)
    node = ColorSubscriber()
    
    running = True
    while running:
        # Handle window events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        
        # Update the screen with the current color
        screen.fill(node.color)
        pygame.display.flip()

        # Spin the ROS node to handle incoming messages
        rclpy.spin_once(node)
    
    node.destroy_node()
    rclpy.shutdown()
    pygame.quit()

if __name__ == '__main__':
    main()

