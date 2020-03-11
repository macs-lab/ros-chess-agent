#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from collections import OrderedDict

board_fen = {'Inital':'rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR','Capturing':'rnbqkb1r/ppp2ppp/4pn2/8/2p5/1P3N2/PB1PPPPP/RN1QKB1R',
'Castling':'r1bq1rk1/p2nbppp/1p2pn2/2p5/Q1P5/2N1PN2/PB1PBPPP/R3K2R','Promotion':'8/1P3k2/8/1p5P/8/8/4p3/1K6'}

def talker():
        pub = rospy.Publisher('Chess_board_info', String, queue_size=70)
        rospy.init_node('Testing', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        for key,info in board_fen.items():
                rospy.loginfo(key)
                pub.publish(info)
                rate.sleep()

if __name__ == '__main__':
        try:
                talker()
                rospy.spin()
        except rospy.ROSInterruptException:
                pass