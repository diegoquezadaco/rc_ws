import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/acis/robotic_cell_files/rc_ws/src/robotic_cell/install/robotic_cell'
