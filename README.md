# ros-chess-agent
## Project Overview
The project is about designing a system that drives the robotic arm to play the physical chess game autonomously with a human opponent. The system is based on an industrial-grade robot manipulator, a computer vision system, an open-source chess engine, and a motion planning algorithm. When playing against a human player, the robotic arm system is able to identify the situation on the chessboard by the computer vision system. The open-source chess engine receives chessboard information and searches the best move.  The motion planning algorithm generates the information of actions that makes the robotic arm moves the chess piece and sent those comments to the robotic arm’s controller to execute the movement.
## Project Introduction 
The system uses Python and ROS to build up the main structure. The system is divided into three nodes: AI node, detecting node, and execution node. 
* The AI node is receiving the chessboard information published by detecting node, searching for the best move, and posting the message about the best move. 
* The detecting node is processing the chessboard picture taking from the camera, recognizing the identity and the position of each piece, transforming the information to the messages that be used in AI node, and publishing them to the master. 
* The execution node is processing the messages published by the AI node, calculating how the robotic arm should physically move, and posting the comments that can be understood by the robot's controller.
