#!/usr/bin/env python
# -*- coding: utf8 -*-
# Insertar una licencia fantabulosa

import rospy
import roslib; roslib.load_manifest('gazebo_ros')
from std_srvs.srv import Empty
from gazebo_msgs.srv import GetModelState
from std_msgs.msg import Float64
import numpy as np
import sys
import os
import time
import random
from gi.repository import Gtk
import ga
import threading

class ControlGui:
	def __init__(self):
		self.builder = Gtk.Builder()
		self.builder.add_from_file('gui/HexapodControl.glade')
		self.window = self.builder.get_object('MainWindow')
		self.n_population = self.builder.get_object('SpinPopulation')
		self.n_generations = self.builder.get_object('SpinGenerations')
		self.n_steps = self.builder.get_object('SpinSteps')
		self.sampling_time = self.builder.get_object('SpinSamplingTime')
		self.p_cross = self.builder.get_object('SpinPCross')
		self.p_mut = self.builder.get_object('SpinPMut')
		self.status_bar = self.builder.get_object('StatusBar')
		self.context_id = self.status_bar.get_context_id("Global Status Bar")

		self.window.show()

		handlers = {
			"on_MainWindow_destroy": Gtk.main_quit,
			"on_stand": sp.stand,
			"on_rest": sp.rest,
			"on_walk": sp.walk,
			"on_reset": sp.reset_pose,
			"on_ButtonStartExperiment_clicked": sp.start_thread,
			"on_ButtonEndExperiment_clicked": sp.end_thread
		}

		self.builder.connect_signals(handlers)

		self.push_message("Status: Standby")

	def push_message(self, message):
		self.status_bar.push(self.context_id, message)

class SpiderControl:
	def __init__(self):
		# Initializes proxy for calling gazebo/reset_simulation service with Empty message response
		rospy.wait_for_service('/gazebo/reset_simulation')		
		try:
			self.resetSimulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

		# Initializes proxy for calling gazebo/get_model_states service with GetModelState message response
		rospy.wait_for_service('/gazebo/get_model_state')
		try:
			self.getModelState = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

		# Initializes joints controllers
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

		self.is_learning = False

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
			print "Brunaldo, que no se te olvide programar bien la caminata."

	def reset_pose(self, event):
		print "Resetting simulation"
		self.resetSimulation()
		gait = []
		for i in range(10):
			gait.append([int(6*random.random()) for i in range(6)])
		print gait
		for k in range(len(gait)):
			self.publish_gait_state(gait[k])
			time.sleep(.2)
		#self.publish_gait_state([0, 1, 2, 3, 4, 5])
		res = self.getModelState('spider', '')
		print "Position x: ", res.pose.position.x
		self.reset_joints()
		time.sleep(.5)
		self.resetSimulation()

	def publish_gait_state(self, gait):
		# gait must be a list of six values for each leg state
		poses = {
				'0': [.5, .25, .25],
				'1': [.7, .25, .25],
				'2': [.7, .20, .25],
				'3': [.5, .20, .25],
				'4': [.3, .20, .25],
				'5': [.3, .25, .25]
			}
		for i in range(len(gait)):
			# publish joints states according to gait states vector
			# (Need to map a gait state to three joint states per leg)
			# Six states from Belter D., Skrzypczynski N. (2010) "A biologically inspired approach to 
			# feasible gait learning for a hexapod robot" will be used
			# [0]: support phase, neutral position: the tip of the leg on the ground
			# [1]: support phase, posterior extreme position: the tip of the leg on the ground
			# [2]: transfer phase, posterior extreme position: the tip of the leg above the ground
			# [3]: transfer phase, neutral position: the tip of the leg above the ground
			# [4]: transfer phase, anterior extreme position: the tip of the leg above the ground
			# [5]: support phase, anterior extreme position: the tip of the leg on the ground
			self.legs[i].publish(poses[str(gait[i])])
			#print "Publicando para ",i, " las poses ",poses[str(gait[i])]

	def spawn_robot(self):
		os.system("rosrun gazebo_ros spawn_model -param robot_description -urdf -model spider")
	def remove_robot(self):
		os.system("rosservice call gazebo/delete_model '{model_name: spider}'")

	def reset_joints(self):
		#print 'Resetting joints'
		if not rospy.is_shutdown():
			self.legs[0].publish([.5, 0, 0])
			self.legs[1].publish([.5, 0, 0])
			self.legs[2].publish([.5, 0, 0])
			self.legs[3].publish([.5, 0, 0])
			self.legs[4].publish([.5, 0, 0])
			self.legs[5].publish([.5, 0, 0])
	def learning_thread(self):
		"""Función encargada del aprendizaje de gaits"""
		print "[Hilo: ",threading.currentThread().getName(), 'Lanzado]'

		n_population = int(gui.n_population.get_value())
		n_generations = int(gui.n_generations.get_value())
		n_steps = int(gui.n_steps.get_value())
		sampling_time = gui.sampling_time.get_value()
		p_cross = gui.p_cross.get_value()
		p_mut = gui.p_mut.get_value()

		gait_population = []
		for i in range(n_population):
			gait = [int(6*random.random()) for i in range(6*n_steps)]
			gait_population.append(gait)

		gen = ga.GeneticAlgorithm(gait_population)
		gen.set_pcross(p_cross)
		gen.set_pmut(p_mut)
		for i in range(n_generations):
			if not self.is_learning:				
				break

			print "Generación: ", i
			fitness = []
			for j in range(len(gait_population)):
				if not self.is_learning:				
					break
				self.resetSimulation
				for k in range(n_steps):
					gait_state = gait_population[j][k*6:k*6+6]
					self.publish_gait_state(gait_state)
					time.sleep(sampling_time)

				res = self.getModelState('spider', '')
				x = res.pose.position.x
				if x < 0:
					fit_val = 0
				else:
					fit_val = (x*1000)**2
				fitness.append(fit_val)
				
				self.reset_joints()
				time.sleep(.5)
				self.resetSimulation()
				print ">Individuo ", j," x: ", x
				status_message = "Status: Learning " + "   Generation: " + str(i+1) + "/" + str(n_generations) + "   Individual: " + str(j+1) + "/" + str(n_population) + "    X: " + str(x*100) + " cms."
				gui.push_message(status_message)
			max_fit = max(fitness)
			index = fitness.index(max_fit)
			print "Generación ",i," finalizada."
			print ">> Mejor distancia: ", max_fit
			print ">> Posición: ", index
			print ">> Patrón: ", gait_population[index]
			gait_population = gen.reproduce(fitness)			

		print "[Hilo: ",threading.currentThread().getName(), 'Deteniendo]'
		status_message = "Status: Standby"
		gui.push_message(status_message)

	def start_thread(self, event):
		# Iniciar el hilo de aprendizaje
		self.is_learning = True
		self.t = threading.Thread(target=self.learning_thread, name="Learning")
		self.t.start()

	def end_thread(self, event):
		# Finalizar el hilo de aprendizaje
		print "Aborting learning process"
		self.is_learning = False		

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