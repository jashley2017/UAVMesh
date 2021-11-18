#!path-to-python3 #TODO
import os
import time
import serial
import sys

import rclpy
from rclpy.node import Node
from environ_msgs.msg import iMET_msg
from networked_sensor.networked_sensor import Sensor # TODO: need sensor package


class iMET(Sensor):
	def __init__(self):
		super().__init__('iMET')
		self.declare_parameter('iMET_topic', 'iMET_data')
		self.iMET_pub = self.create_publisher(IMET_msg, self.get_parameter('iMET_topic').value, 10)
		self.declare_parameter('iMET_port', '/dev/iMET1')
		self.declare_parameter('iMET_baud', '9600')
		#TODO: double check baud, may need to add parity and stop bits arguments
		iMET1 = serial.Serial(self.get_parameter('iMET_port').value, self.get_parameter('iMET_baud').value)
		self.read_thread = threading.Thread(target=self.log_Imet, args=(iMET1,)) #TODO
		self.running = True
		self.read_thread.start()
	
	def __del__(self):
		self.running = False
		self.read_thread.join()

	def log_iMET(self, usb_device):
		while rclpy.ok() and self.running:
			try:
				iMET_raw = usb_dev.readline().decode("ascii")
				parsed = ','.split(iMET_raw)
				sample_time = datetime.strptime(parsed[5] + ',' + parsed[6], '%Y/%m/%d,%H:%M:%S').timestamp()
				ros_iMET_msg = IMET_msg()

				if iMET_attrs[0] == 'XQ':
					ros_iMET_msg.pressure = parsed[1]
					ros_iMET_msg.temp = parsed[2]
					ros_iMET_msg.rel_humidity = parsed[3]
					ros_iMET_msg.humidity_temp = parsed[4]
					ros_iMET_msg.longitude = parsed[7]
					ros_iMET_msg.latitude = parsed[8]
					ros_iMET_msg.altitude = parsed[9]
					ros_iMET_msg.num_sat = parsed[10]
				else:
					self.get_logger().warn("bad IMET message")
				
				ros_iMET_msg.header.stamp.sec = int(sample_time)
				ros_iMET_msg.header.stamp.nsec = int((sample_time - int(sample_time))*1000000000)

				self.iMET_pub(ros_iMET_msg)

		return None

def main(args=None):
	print("launching")
	rclpy.init(args=args)
	iMET_pub = iMET()
	rclpy.spin(iMET_pub)
	iMET_pub.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()

