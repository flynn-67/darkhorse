import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/wego/wego_ws/src/darkhorse/install/smart_dispatcher'
