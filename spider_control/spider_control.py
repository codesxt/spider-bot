#!/usr/bin/env python
# Insertar una licencia fantabulosa

import rospy
from std_msgs.msg import Float64
import numpy as np
import sys
import os
import time
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
			"on_walk": sp.walk,
			"on_reset": sp.reset_pose
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
					'/spider/FR_c_joint_position_controller/command',
					'/spider/ML_a_joint_position_controller/command',
					'/spider/ML_b_joint_position_controller/command',
					'/spider/ML_c_joint_position_controller/command',
					'/spider/MR_a_joint_position_controller/command',
					'/spider/MR_b_joint_position_controller/command',
					'/spider/MR_c_joint_position_controller/command']
		try:
			self.publishers = []
			self.legs = []
			self.legs.append(LegControl('FL', [topics[6], topics[7], topics[8]]))
			self.legs.append(LegControl('FR', [topics[9], topics[10], topics[11]]))
			self.legs.append(LegControl('ML', [topics[12], topics[13], topics[14]]))
			self.legs.append(LegControl('MR', [topics[15], topics[16], topics[17]]))
			self.legs.append(LegControl('BL', [topics[0], topics[1], topics[2]]))
			self.legs.append(LegControl('BR', [topics[3], topics[4], topics[5]]))

			for i in range(len(topics)):
				self.publishers.append(rospy.Publisher(topics[i], Float64, queue_size=10))
			rospy.init_node('spider_control', anonymous=True)
		except rospy.ROSInterruptException:
			pass

	def stand(self, event):
		print 'Standing'
		if not rospy.is_shutdown():
			self.legs[0].publish([.25, .25, .25])
			self.legs[1].publish([.25, .25, .25])
			self.legs[2].publish([.5, .25, .25])
			self.legs[3].publish([.5, .25, .25])
			self.legs[4].publish([.75, .25, .25])
			self.legs[5].publish([.75, .25, .25])
	def rest(self, event):
		print 'Resting'
		if not rospy.is_shutdown():
			self.legs[0].publish([.25, .25, .75])
			self.legs[1].publish([.25, .25, .75])
			self.legs[2].publish([.5, .25, .75])
			self.legs[3].publish([.5, .25, .75])
			self.legs[4].publish([.75, .25, .75])
			self.legs[5].publish([.75, .25, .75])
	def walk(self, event):
		print 'Walking'
		if not rospy.is_shutdown():
			#FL-MR-BL
			self.legs[0].publish([.25, .25, .25])
			self.legs[3].publish([.5, .25, .25])
			self.legs[4].publish([.75, .25, .25])
			#FR-ML-BR
			self.legs[1].publish([.25, .10, .25])
			self.legs[2].publish([.5, .10, .25])
			self.legs[5].publish([.75, .10, .25])

			time.sleep(1)

			#FL-MR-BL
			self.legs[0].publish([.25, .25, .25])
			self.legs[3].publish([.5, .25, .25])
			self.legs[4].publish([.75, .25, .25])
			#FR-ML-BR
			self.legs[1].publish([.25, .10, .25])
			self.legs[2].publish([.25, .10, .25])
			self.legs[5].publish([.25, .10, .25])

			time.sleep(1)

			#FL-MR-BL
			self.legs[0].publish([.25, .25, .25])
			self.legs[3].publish([.5, .25, .25])
			self.legs[4].publish([.75, .25, .25])
			#FR-ML-BR
			self.legs[1].publish([.25, .25, .25])
			self.legs[2].publish([.25, .25, .25])
			self.legs[5].publish([.25, .25, .25])

			time.sleep(1)

			#FL-MR-BL
			self.legs[0].publish([.25, .10, .25])
			self.legs[3].publish([.5, .10, .25])
			self.legs[4].publish([.75, .10, .25])
			#FR-ML-BR
			self.legs[1].publish([.5, .25, .25])
			self.legs[2].publish([.5, .25, .25])
			self.legs[5].publish([.75, .25, .25])

			time.sleep(1)

			#FL-MR-BL
			self.legs[0].publish([.25, .25, .25])
			self.legs[3].publish([.25, .25, .25])
			self.legs[4].publish([.25, .25, .25])
			#FR-ML-BR
			self.legs[1].publish([.5, .25, .25])
			self.legs[2].publish([.5, .25, .25])
			self.legs[5].publish([.75, .25, .25])
	def reset_pose(self, event):
		print "Resetting pose"
		

class LegControl:
	'Class to control a leg of the robot'
	legs = 0
	def __init__(self, name, topics):
		LegControl.legs += 1
		self.name = name
		self.publishers = []
		for i in range(len(topics)):
			self.publishers.append(rospy.Publisher(topics[i], Float64, queue_size=10))

	def publish(self, values):
		values = self.map(values)
		for i in range(len(values)):
			self.publishers[i].publish(values[i])

	def map(self, values):
		values[0] = self.maprange((0, 1), (-1.57,1.57), values[0])		
		if self.name[1] == 'L':
			values[1] = self.maprange((0, 1), (0,-3.14), values[1])
			values[2] = self.maprange((0, 1), (0,-3.14), values[2])
		else:
			values[1] = self.maprange((0, 1), (0,3.14), values[1])
			values[2] = self.maprange((0, 1), (0,3.14), values[2])			
		return values

	def maprange(self, a, b, s):
		(a1, a2), (b1, b2) = a, b
		return  b1 + ((s - a1) * (b2 - b1) / (a2 - a1))

if __name__ == '__main__':
	rDir = os.path.split(sys.argv[0])[0]
	os.chdir(rDir)

	sp = SpiderControl()
	gui = ControlGui()
	Gtk.main()