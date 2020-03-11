# Robot-plays-chess

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

Due to the python version requirement for using python-chess package, the environment for running ROS node need to be python3 so the first line is ```#!/usr/bin/env python3```
  
The general information published by this node is a string that contains
Move(4 letters),is castling(yes or no),is capturing(yes or no)
For promotion

In move part string, the first two letters present which piece on that position is going to move. The last two letters present the position of the piece will arrive.

Capturing is that a piece takes down an opponent’s piece


Castling is a tactic in chess. It can only be applied under the following condition. When one of the rooks and king is at their original position i.e. king and rook do not move after the game starts. Between them, there is no other piece. Then, the king can move to either g or c and the rook on h will jump to f or rook on a will jump to d
 
