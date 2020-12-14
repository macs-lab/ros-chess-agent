#!/usr/bin/env python3
import chess
import chess.engine
import rospy
from std_msgs.msg import String
from chess_ai.srv import ai_service, ai_serviceResponse
import os
import sys

def open_chess_engine():
    path = os.getcwd()
    path = os.path.abspath(os.path.join(path, os.pardir))
    #engine_path = path+'/stockfish/stockfish-11-linux/Linux/stockfish_20011801_x64'
    engine_path = '/home/mingyu/mingyu_ws/src/ros-chess-agent/chess_ai/src/stockfish/stockfish-11-linux/Linux/stockfish_20011801_x64'
    engine = chess.engine.SimpleEngine.popen_uci(engine_path)
    extreme_setup = {'Threads': 10, 'Hash': 5000, 'Skill Level': 20,}
    engine.configure(extreme_setup)
    return engine

def service_handle(msg):
    board = chess.Board(msg.chess_board_state+ ' w KQkq - 0 1')
    time_limit = chess.engine.Limit(time = 0.1)
    if not board.is_game_over():
        action = engine.play(board,time_limit)
        chess_move = str(action.move)
        if board.is_capture(action.move): result = chess_move +',yes'+',no'
        elif board.is_castling(action.move): result = chess_move +',no'+',yes'
        elif len(chess_move) == 5: result = chess_move[:-1] +',no'+',no' + ',' + chess_move[-1]
        else: result = chess_move +',no'+',no'
        rospy.loginfo(msg.chess_board_state)
        rospy.loginfo(result)
        return ai_serviceResponse(result)
    else: ai_serviceResponse("Game is over")

if __name__ =="__main__":
    try:
        rospy.init_node('Chess_AI')
        engine = open_chess_engine()
        rospy.loginfo("starting service.....")
        s = rospy.Service('chess_ai_service', ai_service, service_handle)
        rospy.loginfo("chess ai service started")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass