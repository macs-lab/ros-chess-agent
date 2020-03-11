#!/usr/bin/env python3
import chess
import chess.engine
import rospy
from std_msgs.msg import String

def open_chess_engine():
    engine = chess.engine.SimpleEngine.popen_uci("~/Documents/stockfish-11-linux/Linux/stockfish_20011801_x64")
    extreme_setup = {'Threads': 10, 'Hash': 5000, 'Skill Level': 20,}
    engine.configure(extreme_setup)
    return engine

def decision(msg):

    board = chess.Board(msg.data + ' w KQkq - 0 1')
    time_limit = chess.engine.Limit(time = 0.1)
    if not board.is_game_over():
        action = engine.play(board,time_limit)
        chess_move = str(action.move)
        if board.is_capture(action.move): result = chess_move +',yes'+',no'
        elif board.is_castling(action.move): result = chess_move +',no'+',yes'
        elif len(chess_move) == 5: result = chess_move[:-1] +',no'+',no' + ',' + chess_move[-1]
        else: result = chess_move +',no'+',no'
        rospy.loginfo(msg.data)
        rospy.loginfo(result)
        move.publish(result)
    else: move.publish("Game is over")


if __name__ =="__main__":
    try:
        rospy.init_node('Chess_AI')
        engine = open_chess_engine()
        rospy.Subscriber('Chess_board_info', String, decision)
        move = rospy.Publisher('AI_move', String, queue_size=10)
    except rospy.ROSInterruptException:
        pass