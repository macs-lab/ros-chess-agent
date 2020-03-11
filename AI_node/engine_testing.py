import chess
import chess.engine

engine_stock = chess.engine.SimpleEngine.popen_uci("~/Documents/stockfish-11-linux/Linux/stockfish_20011801_x64")
extreme_setup = {'Threads': 10, 'Hash': 5000, 'Skill Level': 20,}
engine_stock.configure(extreme_setup)

engine_komodo = chess.engine.SimpleEngine.popen_uci("~/Documents/komodo-10_ae4bdf/Linux/komodo-10-linux")

board = chess.Board()
total_action = 0

while not board.is_game_over():
        if total_action % 2 == 0:
                print('player: stockfish')
                action = engine_stock.play(board, chess.engine.Limit(time = 0.5))
        else:
                print('player: komodo')
                action = engine_komodo.play(board, chess.engine.Limit(time = 0.5))
        board.push(action.move)
        print(board)
        total_action += 1

if (total_action-1) % 2 == 0: print('stockfish win')
else: print('komodo win')
