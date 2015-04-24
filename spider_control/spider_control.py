#!/usr/bin/env python
# Insertar una licencia fantabulosa

import rospy
from std_msgs.msg import Float64
import numpy as np
import sys
import os
from gi.repository import Gtk

class ControlGui:
	def __init__(self):
		self.builder = Gtk.Builder()
		self.builder.add_from_file('gui/spider.glade')
		self.window = self.builder.get_object('MainWindow')

		self.window.show()

		handlers = {
			"on_MainWindow_destroy": Gtk.main_quit,
			"on_stand": sp.stand,
			"on_rest": sp.rest,
			"on_walk": sp.walk
		}

		self.builder.connect_signals(handlers)

class SpiderControl:
	def __init__(self):
		topics = [	'/spider/BL_a_joint_position_controller/command',
					'/spider/BL_b_joint_position_controller/command',
					'/spider/BL_c_joint_position_controller/command',
					'/spider/BR_a_joint_position_controller/command',
					'/spider/BR_b_joint_position_controller/command',
					'/spider/BR_c_joint_position_controller/command',
					'/spider/FL_a_joint_position_controller/command',
					'/spider/FL_b_joint_position_controller/command',
					'/spider/FL_c_joint_position_controller/command',
					'/spider/FR_a_joint_position_controller/command',
					'/spider/FR_b_joint_position_controller/command',
					'/spider/FR_c_joint_position_controller/command']
		try:
			self.publishers = []
			for i in range(len(topics)):
				self.publishers.append(rospy.Publisher(topics[i], Float64, queue_size=10))
			rospy.init_node('spider_control', anonymous=True)
		except rospy.ROSInterruptException:
			pass

		# FL_a {-1.57: atras, 1.75: adelante}
		# FL_b {-3.14: abajo, 0: arriba}
		# FL_c {-3.14: extendida, 0: contraida}		

	def stand(self, event):
		print 'Standing'
		if not rospy.is_shutdown():
			self.publishers[6].publish(-1)
			self.publishers[7].publish(-1)
			self.publishers[8].publish(-1)

			self.publishers[9].publish(-1)
			self.publishers[10].publish(1)
			self.publishers[11].publish(1)

			self.publishers[0].publish(1)
			self.publishers[1].publish(-1)
			self.publishers[2].publish(-1)

			self.publishers[3].publish(1)
			self.publishers[4].publish(1)
			self.publishers[5].publish(1)
	def rest(self, event):
		print 'Resting'
		if not rospy.is_shutdown():
			self.publishers[8].publish(-2)
			self.publishers[11].publish(2)
			self.publishers[2].publish(-2)
			self.publishers[5].publish(2)

	def walk():
		print 'Walking'

if __name__ == '__main__':
	rDir = os.path.split(sys.argv[0])[0]
	os.chdir(rDir)

	sp = SpiderControl()
	gui = ControlGui()
	Gtk.main()