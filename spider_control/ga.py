# -*- coding: utf8 -*-
import random

class GeneticAlgorithm:
	population = []
	fitness = []
	population_size = 0
	cross_prob = 0
	mut_prob = 0
	def __init__(self, population):
		"""
		Constructor para algoritmos genéticos con población predefinida.
		"""
		self.population = population
		self.population_size = len(population)
		self.cross_prob = 0.6
		self.mut_prob = 0.01
		print "Algoritmo genético inicializado con población de largo: ", self.population_size

	def set_fitness(self, fitness):
		"""
		Se recibe un vector fitness que se asigna como segunda columna a la población.
		"""
		if (len(fitness) != self.population_size):
			print "El largo del vector de fitness no es el mismo que el de la población."
			print self.population_size
			return
		else:
			self.fitness = fitness

	def set_pcross(self, pcross):
		self.cross_prob = pcross

	def set_pmut(self, pmut):
		self.mut_prob = pmut

	def print_population(self):
		for i in range(self.population_size):
			print self.population[i], "[Fitness: ", self.fitness[i],"]"

	def get_population(self):
		return self.population

	def roulette_select(self):
		"""
		Roulette selection of fitter individuals based on fitness.
		"""
		new_population = []
		N = self.population_size
		w_max = max(self.fitness)
		index = random.randint(0,N-1)
		B = 0
		for i in range(N):
			B = B + random.random()*2.0*w_max
			while (B > self.fitness[index]):
				B -= self.fitness[index]
				index = (index+1)%N
			new_population.append(self.population[index])
		return new_population

	def crossover(self):
		for i in range(self.population_size/2):
			if (random.random() < self.cross_prob):
				size = len(self.population[0])
				cut_point = random.randint(0,size-1)				
				i1 = self.population[2*i]
				i2 = self.population[2*i+1]
				h1 = i1[0:cut_point] + i2[cut_point:size]
				h2 = i2[0:cut_point] + i1[cut_point:size]
				self.population[2*i] = h1
				self.population[2*i+1] = h2
		return self.population

	def mutate(self):
		for i in range(self.population_size):
			if (random.random() < self.mut_prob):
				mut_point = random.randint(0,len(self.population[i])-1)
				self.population[i][mut_point] = random.randint(0,5)
		return self.population

	def reproduce(self, fitness):
		self.set_fitness(fitness)
		self.population = self.roulette_select()
		self.population = self.crossover()
		self.population = self.mutate()
		return self.population
