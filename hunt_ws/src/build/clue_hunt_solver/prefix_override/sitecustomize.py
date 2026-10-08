import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/swarnava2/hunt_ws/src/install/clue_hunt_solver'
