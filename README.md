This package serves as a tool for testing and debugging PDDL files without the need for a simulator or any action servers. By utilizing a graphical user interface (GUI) and user input, predicates and results to actions are set, which facilitates a faster and easier debugging process. This approach allows for quick modifications and testing of different scenarios.

 You need to clone the PDDL_Executor repo: git clone https://github.com/olaghattas/PDDL-Executor


TODO: Instructions
to run GUI

ros2 run pddl_debugger_interface gui_interface

note that you should be answering the question form the gui.
These are ros services.

to run the planner
ros2 run shr_plan_debug planning_controller_node 

there is some logic that is already taken care of in the planner.
For example, at start the robot location will be set to home. When planner hits undocking, you can see the gui turn robot charging to false.


