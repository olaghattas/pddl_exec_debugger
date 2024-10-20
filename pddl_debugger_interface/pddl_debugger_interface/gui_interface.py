import sys
import threading
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QLabel, QVBoxLayout, QHBoxLayout, QComboBox, QSpinBox,QLineEdit, QFrame
from PyQt5.QtCore import Qt
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
from builtin_interfaces.msg import Time
from gui_interfaces.srv import ActionReq

class TrueFalseGroup(QWidget):
    def __init__(self, node):
        super().__init__()

        # Set window title
        self.setWindowTitle('Group of Publishers')

        # Create the main layout (vertical)
        main_layout = QVBoxLayout()

        ##### Start robot location dropdown ###########
        # Add a label (text) for the dropdown
        label_location_rob = QLabel('Select the robot\'s current location:')
        main_layout.addWidget(label_location_rob)
        # Create the dropdown (combo box) for location selection
        self.location_robot_dropdown = QComboBox()
        self.location_robot_dropdown.addItems(['', 'living_room', 'home', 'outside', 'bedroom'])
        # self.location_robot_dropdown.setCurrentText('living_room')

        # Add the dropdown to the main layout
        main_layout.addWidget(self.location_robot_dropdown)
        ##### End robot location dropdown ###########


        ##### Start person location dropdown ###########
        # Add a label (text) for the dropdown
        label_location_person = QLabel('Select the person\'s current location:')
        main_layout.addWidget(label_location_person)
        # Create the dropdown (combo box) for location selection
        self.location_person_dropdown = QComboBox()
        self.location_person_dropdown.addItems(['', 'living_room', 'outside', 'bedroom'])
        # Add the dropdown to the main layout
        main_layout.addWidget(self.location_person_dropdown)
        ##### End person location dropdown ###########

        # Store references to the publishers and last pressed buttons
        self.publishers = {}
        self.last_pressed_button = {}

        # Define the questions and topics
        questions = [
            ('Did the person take their medicine?', 'person_taking_medicine'),
            ('Is the person eat?', 'person_eating'),
            ('Is the robot charging?', 'charging')
        ]

        # Create widgets for each question
        for question, topic in questions:
            self.create_question_section(main_layout, question, topic, node)


        ## publishers for dropdown
        self.publisher_robot_location = node.create_publisher(String, 'robot_at', 10)
        # Connect the dropdown selection change to the function that publishes the location
        self.location_robot_dropdown.currentIndexChanged.connect(lambda: self.publish_location(1))

        self.publisher_person_location = node.create_publisher(String, 'person_at', 10)
        # Connect the dropdown selection change to the function that publishes the location
        self.location_person_dropdown.currentIndexChanged.connect(lambda: self.publish_location(0))

        ### time publisher
        # Create a horizontal layout for hours and minutes inputs
        time_input_layout = QHBoxLayout()

        # Create label for hours input
        label_hours = QLabel('Hours:')
        time_input_layout.addWidget(label_hours)

        # Create a SpinBox for hours (24-hour format: 0 to 23)
        self.hours_spinbox = QSpinBox()
        self.hours_spinbox.setRange(0, 23)  # 24-hour format
        time_input_layout.addWidget(self.hours_spinbox)

        # Create label for minutes input
        label_minutes = QLabel('Minutes:')
        time_input_layout.addWidget(label_minutes)

        # Create a SpinBox for minutes (0 to 59)
        self.minutes_spinbox = QSpinBox()
        self.minutes_spinbox.setRange(0, 59)
        time_input_layout.addWidget(self.minutes_spinbox)

        # Add the time input layout to the main layout
        main_layout.addLayout(time_input_layout)
        # Connect the spin box value change to the publish function
        self.hours_spinbox.valueChanged.connect(self.publish_time)
        self.minutes_spinbox.valueChanged.connect(self.publish_time)

        # ROS 2 publisher
        self.publisher_time = node.create_publisher(Time, '/protocol_time', 10)


        #### service to repomnde to action interface

        # Create a status indicator (a small circle)
        self.status_indicator = QFrame(self)
        self.status_indicator.setFixedSize(20, 20)  # Small size
        self.status_indicator.setStyleSheet("border-radius: 10px; background-color: gray;")  # Gray circle
        main_layout.addWidget(self.status_indicator)

        self.service = node.create_service(ActionReq, 'input_request', self.handle_request)
        # Add a label to display the question
        self.question_label = QLabel("Question will appear here.")
        main_layout.addWidget(self.question_label)

        # Create a line edit for the user to input their response
        self.response_input = QLineEdit()
        self.response_input.setPlaceholderText("Type your response here...")
        main_layout.addWidget(self.response_input)

        # Create a button to submit the response
        submit_button = QPushButton('Submit Response')
        submit_button.clicked.connect(self.submit_response)
        main_layout.addWidget(submit_button)
        self.flag = False
        self.handle_flag = False

        # Set the main layout for the window
        self.setLayout(main_layout)


    def submit_response(self):
        # Get the user's response from the input box
        user_response = self.response_input.text()
        print("User response sent:", user_response)

        if self.handle_flag:
            self.flag = True

        self.response = user_response

        # Clear the input box and question label after submitting
        self.response_input.clear()
        self.question_label.setText("Waiting for next question...")
        # Reset the status indicator to its original color (gray)
        self.status_indicator.setStyleSheet("border-radius: 10px; background-color: gray;")


    def handle_request(self, request, response):
        question = request.question
        print("question", question)

        ## so submit repsonse flag is only valid if we recive a request
        self.handle_flag = True
        self.question_label.setText(question)  # Display the question
        # Change the status indicator color to green when new text arrives
        self.status_indicator.setStyleSheet("border-radius: 10px; background-color: green;")

        while not self.flag:
            # print("DIDNT RECIVE RESPONSE YET")
            ## Wait until you recieve a response
            pass

        response.response = self.response
        print("self.flag",self.flag)
        self.flag = False
        self.handle_flag = False

        return response

    def publish_location(self, robot):
        # Get the selected location from the dropdown
        if robot:
            selected_location = self.location_robot_dropdown.currentText()
        else:
            selected_location = self.location_person_dropdown.currentText()

        # Log the selected location
        # node.get_logger().info(f'Publishing location: {selected_location}')

        # Create and publish the message
        msg = String()
        msg.data = selected_location
        if robot:
            self.publisher_robot_location.publish(msg)
        else:
            self.publisher_person_location.publish(msg)



    def create_question_section(self, layout, question_text, topic_name, node):
        # Add a label (text) before the buttons
        label = QLabel(question_text)
        layout.addWidget(label)

        # Create a horizontal layout for the buttons (side by side)
        button_layout = QHBoxLayout()

        # Create buttons
        true_button = QPushButton('True')
        false_button = QPushButton('False')

        # Add buttons to the horizontal layout
        button_layout.addWidget(true_button)
        button_layout.addWidget(false_button)

        # Add the horizontal layout (buttons) to the main layout
        layout.addLayout(button_layout)

        # Connect buttons to their functions
        true_button.clicked.connect(lambda: self.publish_value(1, topic_name, true_button, false_button))
        false_button.clicked.connect(lambda: self.publish_value(0, topic_name, true_button, false_button))

        # Create a publisher for this question's topic
        self.publishers[topic_name] = node.create_publisher(Int32, topic_name, 10)

        # Initialize the last pressed button dictionary
        self.last_pressed_button[topic_name] = None

    def publish_value(self, value, topic_name, true_button, false_button):
        # Log the pressed value
        # self.publishers[topic_name].get_logger().info(f'{topic_name} value: {value == 1}')
        
        # Create and publish the message
        msg = Int32()
        msg.data = value
        self.publishers[topic_name].publish(msg)

        # Reset styles for both buttons
        true_button.setStyleSheet("")
        false_button.setStyleSheet("")

        # Set the style for the last pressed button
        if value == 1:
            true_button.setStyleSheet("background-color: lightgreen;")
            self.last_pressed_button[topic_name] = true_button
        else:
            false_button.setStyleSheet("background-color: lightcoral;")
            self.last_pressed_button[topic_name] = false_button

    def publish_time(self):
        # Get the selected hours and minutes
        hours = self.hours_spinbox.value()
        minutes = self.minutes_spinbox.value()


        # Convert the time to seconds
        time_in_seconds = (hours * 3600) + (minutes * 60)

        # Publish the time in seconds
        msg = Time()
        msg.sec = time_in_seconds
        msg.nanosec = 0
        self.publisher_time.publish(msg)


def main(args=None):
    # Initialize ROS 2
    rclpy.init()

    # Create a simple ROS 2 node directly
    node = Node('true_false_node')

    # Create a thread for running rclpy.spin() in the background
    ros2_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros2_thread.start()

    # Create the Qt application
    app = QApplication(sys.argv)

    # Create a grouped window for True/False questions
    grouped_window = TrueFalseGroup(node)

    # Show the window
    grouped_window.show()

    # Run the Qt event loop
    sys.exit(app.exec_())

    # Shutdown ROS 2 after the app is closed
    rclpy.shutdown()

if __name__ == '__main__':
    main()