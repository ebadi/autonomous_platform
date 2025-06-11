import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/root/ap4_hlc_docker_dir/ap4hlc_ws/install/path_tracker'
