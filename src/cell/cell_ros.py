#!/usr/bin/env python3

from IPython.core.debugger import set_trace
import rospy
from std_msgs.msg import String
import numpy as np
import socket
from cell_estimator.msg import imu
from cell_estimator.msg import RelPos
from cell_estimator.msg import PositionVelocityTime
#from ublox.msg import PositionVelocityTime
from listener import udp_receiver


def cell_ros():

# 	# # Start UDP port
# 	# udp_port = 1234
# 	# buffer_size = 2048
# 	# sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# 	# sock.bind(('', udp_port))
# 	# sock.setblocking(True)
# 	# printer('Opened UDP socket', str(udp_port))

	while not rospy.is_shutdown():
		cell.main_loop()


if __name__ == '__main__':
	rospy.init_node('cell_ros', anonymous=True)
	cell = udp_receiver()
	cell_ros()
	cell.run_receiver()
