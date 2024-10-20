import rclpy
from rclpy.node import Node
from gui_interfaces.srv import ActionReq  # Import your service type
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QLineEdit, QPushButton

class InputServer(Node):
    def __init__(self):
        super().__init__('input_server')

        # Create a ROS 2 service
        self.service = self.create_service(ActionReq, 'input_request', self.handle_request)
        # Create the GUI application
        self.app = QApplication([])
        self.window = QWidget()
        self.window.setWindowTitle('Service Response GUI')

        # Create the main layout
        main_layout = QVBoxLayout()

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

        # Set the main layout for the window
        self.window.setLayout(main_layout)

        # Initialize variables to store the request and response future
        self.current_request = None
        self.current_future = None  # Store the future object for the response

        # Show the window
        self.window.show()
        self.app.exec_()

    def handle_request(self, request, response_future):
        print("Received question:", request.question)
        # Store the future object and request
        self.current_request = request
        self.current_future = response_future  # Future object to be set later

        # Display the question in the GUI
        question = request.question
        print("Received question:", question)
        self.question_label.setText(question)

        # Do not return the response yet, we will set it later when the user provides input

    def submit_response(self):
        # Get the user's response from the input box
        user_response = self.response_input.text()

        # Set the response based on user input and clear the input box
        if self.current_future:  # Check if there is a future waiting to be set
            response = ActionReq.Response()  # Create the response object
            response.response = user_response  # Set the user's response

            # Set the response for the service and clear input
            self.current_future.set_result(response)  # This sends the response back to the client
            print("User response sent:", user_response)

            # Clear the input box and question label after submitting
            self.response_input.clear()
            self.question_label.setText("Waiting for next question...")

            # Reset the stored future and request
            self.current_future = None
            self.current_request = None

def main():
    rclpy.init()

    # Create and spin the ROS 2 service node
    input_server = InputServer()
    rclpy.spin(input_server)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
