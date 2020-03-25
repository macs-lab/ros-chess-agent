# ros-chess-agent
## Project Overview
The project is about designing a system that drives the robotic arm to play the physical chess game autonomously with a human opponent. The system is based on an industrial-grade robot manipulator, a computer vision system, an open-source chess engine, and a motion planning algorithm. When playing against a human player, the robotic arm system is able to identify the situation on the chessboard by the computer vision system. The open-source chess engine receives chessboard information and searches the best move.  The motion planning algorithm generates the information of actions that makes the robotic arm moves the chess piece and sent those comments to the robotic arm’s controller to execute the movement.
## Project Introduction 
The system uses Python and ROS to build up the main structure. The system is divided into three nodes: AI node, detecting node, and execution node. The AI node is receiving the chessboard information published by detecting node, searching for the best move, and posting the message about the best move. The detecting node is processing the chessboard picture taking from the camera, recognizing the identity and the position of each piece, transforming the information to the messages that be used in AI node, and publishing them to the master. The execution node is processing the messages published by the AI node, calculating how the robotic arm should physically move, and posting the comments that can be understood by the robot's controller.
## AI_node.py
This node generates the chess move and publishing it in the ROS master. It first subscribes to the ROS topic that has the present chessboard information expressed in fen string and searches the best move for that present chessboard.

### Engine Download 

The chess engine, stockfish 11, is used for playing the game. The following steps are the procedure of downloading the engine through the terminal. The engine can also be obtained via website

* Cd ```desire folder for saving the engine```
* Run: ```wget https://stockfishchess.org/files/stockfish-11-linux.zip```
* Unzip the file: ```unzip stockfish-11-linux.zip```
* Cd ```/stockfish-11-linux/Linux```
* Run: ```chmod +x stockfish_20011801_x64(or press tab)```
* Testing whether the engine is activated, run: ```./stockfish_20011801_x64```

### Python-chess

The library, python-chess, is a package for communicating with the engine. The latest version is only for python3 so, when download via pip, make sure that pip downloads in correct python version.

* To download, run: ```pip install python-chess```

### Code explanation

Due to the python version requirement for using the python-chess package, the environment for running ROS node need to be python3, so the first line is ```#!/usr/bin/env python3```.
  
The information published by this node is a string. The picture below shows the chess board's fen string that the node received for searching a move and the published message.
![](image/AI_node_publish_info.png)

In general, a message contains move(4 letters), castling(yes or no), capturing(yes or no). For example ```e2e4,no,no```.

* In move part string, the first two letters present which piece on that position is going to move. The last two letters present the position of the piece will arrive.

* Capturing is the action that piece takes down an opponent's piece.

* Castling is a tactic in chess. It can only be applied under the following condition, and it demonstrates by the picture below. When one of the rooks and king is at their original position i.e., king and rook, do not move after the game starts. Between them, there is no other piece. Then, the king can move to either g or c, and the rook on h will jump to f or rook on a will jump to d.

![](image/Castling.png)
 
When a pawn reaches the other side of the chessboard edge, it gets the promotion, which this piece can be replaced by the other piece, for example, queen. It does not matter whether this piece is on the board or not. For this situation, the published message contains move(4 letters), castling(yes or no), capturing(yes or no), and piece type. For example, ```b7b8,no,no,q```, q stands for the queen.
