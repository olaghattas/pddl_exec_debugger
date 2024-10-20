import sys
import threading
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class YesNoWindow(QWidget):
    def __init__(self, node):
        super().__init__()

        # Set window title
        self.setWindowTitle('Yes or No')

        # Create layout
        layout = QVBoxLayout()

        # Create buttons
        yes_button = QPushButton('Yes')
        no_button = QPushButton('No')

        # Add buttons to layout
        layout.addWidget(yes_button)
        layout.addWidget(no_button)

        # Connect buttons to their functions
        yes_button.clicked.connect(self.publish_yes)
        no_button.clicked.connect(self.publish_no)

        # ROS 2 node for publishing
        self.node = node
        self.publisher_pills_motion_sensor = self.node.create_publisher(Int32, 'person_taking_medicine', 10)

        # Set layout
        self.setLayout(layout)

    def publish_yes(self):
        self.node.get_logger().info('pills_motion_sensor true')
        msg = Int32()
        msg.data = 1
        self.publisher_pills_motion_sensor.publish(msg)

    def publish_no(self):
        self.node.get_logger().info('pills_motion_sensor false')
        msg = Int32()
        msg.data = 0
        self.publisher_pills_motion_sensor.publish(msg)


class Ros2Node(Node):
    def __init__(self):
        super().__init__('yes_no_node')


def run_ros2():
    rclpy.init()
    node = Ros2Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    # Create a thread for running the ROS 2 node
    ros2_thread = threading.Thread(target=run_ros2, daemon=True)
    ros2_thread.start()

    # Create the Qt application
    app = QApplication(sys.argv)

    # Initialize the ROS 2 node
    node = Ros2Node()

    # Create instance of the window and pass the ROS 2 node
    window = YesNoWindow(node)

    # Show the window
    window.show()

    # Run the Qt event loop
    sys.exit(app.exec_())
