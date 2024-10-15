import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/soumyajit/Eyantra-CoBot-24/src/install/ur5_control'
