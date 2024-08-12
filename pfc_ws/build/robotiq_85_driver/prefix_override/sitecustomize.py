import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/yuritvr/PFC/pfc_ws/install/robotiq_85_driver'
