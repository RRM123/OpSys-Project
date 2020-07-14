import sys
import math
import time
import random48 as rand

alphabet = {1: 'A', 2: 'B', 3: 'C', 4: 'D', 5: 'E', 6: 'F', 7: 'G', 8: 'H', 9: 'I', 10: 'J', 11: 'K', 12: 'L', 13: 'M', 14: 'N', 15: 'O', 16: 'P', 17: 'Q', 18: 'R', 19: 'S', 20: 'T', 21: 'U', 22: 'V', 23: 'W', 24: 'X', 25: 'Y', 26: 'Z'}
random = rand.Random()

class Process(object):
	def __init__(self, num):
		self.name = alphabet[num]
		self.init_arrival = 0
		self.num_bursts = 0
		self.current_burst = 0
		self.cpu_times = []
		self.io_times = []

	def get_name(self):
		return self.name

	def get_init_arrival(self):
		return self.init_arrival

	def get_num_bursts(self):
		return self.num_bursts

	def get_cpu_io_times(self, burst):
		return self.cpu_times[burst], self.io_times[burst]

	def get_current_burst(self):
		return self.current_burst

	def change_CPU_time(self, burst, new_time):
		self.cpu_times[burst] = new_time

	def increment_burst(self):
		self.current_burst += 1

	def make_bursts(self, lamb, upper_bound):
		self.init_arrival = int(math.floor(random.get_random(lamb, upper_bound)))
		self.num_bursts = int(math.floor(random.drand48()*100) + 1)
		self.cpu_times = [0] * self.num_bursts
		self.io_times = [0] * self.num_bursts

		for i in range(self.num_bursts-1):
			self.cpu_times[i] = math.ceil(random.get_random(lamb, upper_bound))
			self.io_times[i] = math.ceil(random.get_random(lamb, upper_bound))

		self.cpu_times[self.num_bursts-1] = random.get_random(lamb, upper_bound)
		self.io_times[self.num_bursts-1] = None


class Simulation(object):
	def __init__(self):
		self.queue = []
		self.cpu = None
		self.io = {}

	def get_CPU_process(self):
		return self.cpu

	def get_next_process(self):
		if len(self.queue) > 0:
			return self.queue.pop(0)

	def get_io_end_time(self, process):
		return self.io[process]

	def queue_size(self):
		return len(self.queue)

	def addProcessToQueue(self, process, wait_time):
		# wait for wait_time
		# start_time = time.time()
		# while int(time.time() - start_time)*1000 < wait_time:
		# 	continue
		self.queue.append(process)

	def addProcessToCPU(self, process):
		# start_time = time.time()
		# while int(time.time() - start_time) < (cs_time/2):
		# 	continue
		self.cpu = process

	def removeProcessFromCPU(self, process):
		if self.cpu.get_name() == process.get_name():
			# start_time = time.time()
			# while int(time.time() - start_time) < (cs_time/2):
			# 	continue
			self.cpu = None

	def addProcessToIO(self, process, end_time):
		self.io[process] = end_time

	def removeProcessFromIO(self, process):
		self.io[process] = 0

	def print_queue(self):
		print("[Q", end = " ")
		if len(self.queue) == 0:
			print("<empty>]")
		else:
			for i in range(len(self.queue) - 1):
				print(self.queue[i].get_name(), end = " ")
			print(self.queue[len(self.queue)-1].get_name() + "]")

	def reset(self):
		self.queue = []
		self.cpu = None
		self.io = {}

# simulations --------------------------------------------------------------------------------------------

# print functions
def printArrival(timer, name):
	print("time " + str(int(timer)) + "ms: " + "Process " + name + " arrived; added to ready queue", end = " ")

def printCPUStart(timer, name, cpu_time):
	print("time " + str(int(timer)) + "ms: " + "Process " + name + " started using the CPU for " + str(int(cpu_time)) + "ms burst", end = " ")

def printCPUEnd(timer, name, burst_num):
	print("time " + str(int(timer)) + "ms: " + "Process " + name + " completed a CPU burst; " + str(int(burst_num)) + " bursts to go", end = " ")

def printSwitchToIO(timer, name, io_time):
	print("time " + str(int(timer)) + "ms: " + "Process " + name + " switching out of CPU; will block on I/O until time " + str(int(io_time)) + "ms", end = " ")

def process_arrival(process):
	print("Process " + str(process.get_name()) + " [NEW] (arrival time " + str(process.get_init_arrival()) + " ms) " + str(process.get_num_bursts()) + " CPU bursts")
	#print process.arrival()

# main simulation functions
def fcfs():
	print("time 0ms: " + "Simulator started for FCFS [Q <empty>]")
	print("")

def sortByCPUTime(process):
	cpu, io = process.get_cpu_io_times(process.get_current_burst())
	return cpu

def sjf(temp_processes, cs_time, alpha):
	processes = sorted(temp_processes, key = sortByCPUTime)

	sjf_simulation = Simulation()
	

	print("time 0ms: " + "Simulator started for SJF [Q <empty>]")
	print("")

def srt():
	print("time 0ms: " + "Simulator started for SRT [Q <empty>]")
	print("")

def sortByArrivalTime(process):
	return process.get_init_arrival()

def rr(temp_processes, slice_time, cs_time):
	processes = sorted(temp_processes, key = sortByArrivalTime)
	print("time 0ms: " + "Simulator started for RR [Q <empty>]")

	rr_simulation = Simulation()

	current_arrival = 0
	current_cpu_process = rr_simulation.get_CPU_process()
	current_bursts = {}
	for i in range(len(processes)):
		current_bursts[processes[i].get_name()] = 0
	timer = 0
	cpu_start_time = 0

	while True:

		# new arrival
		if current_arrival < len(processes) and processes[current_arrival].get_init_arrival() == timer:
			rr_simulation.addProcessToQueue(processes[current_arrival], timer)
			printArrival(timer, processes[current_arrival].get_name())
			rr_simulation.print_queue()
			current_arrival += 1

		# add to CPU
		if rr_simulation.queue_size() > 0 and current_cpu_process == None:
			timer += (cs_time/2)
			rr_simulation.addProcessToCPU(rr_simulation.get_next_process())
			cpu_start_time = timer
			current_cpu_process = rr_simulation.get_CPU_process()
			printCPUStart(timer, current_cpu_process.get_name(), current_cpu_process.get_cpu_io_times(current_bursts[current_cpu_process.get_name()])[0])
			rr_simulation.print_queue()

		# CPU process done, switch to I/O
		if current_cpu_process != None and cpu_start_time + current_cpu_process.get_cpu_io_times(current_bursts[current_cpu_process.get_name()])[0] == timer:
			rr_simulation.removeProcessFromCPU(current_cpu_process)
			printCPUEnd(timer, current_cpu_process.get_name(), current_cpu_process.get_num_bursts() - current_bursts[current_cpu_process.get_name()] - 1)
			rr_simulation.print_queue()
			rr_simulation.addProcessToIO(current_cpu_process, timer + current_cpu_process.get_cpu_io_times(current_bursts[current_cpu_process.get_name()])[1] + (cs_time/2))
			printSwitchToIO(timer, current_cpu_process.get_name(), rr_simulation.get_io_end_time(current_cpu_process))
			rr_simulation.print_queue()
			current_bursts[current_cpu_process.get_name()] += 1
			current_cpu_process = rr_simulation.get_CPU_process()

		timer += 1
		# testing
		if timer > 67:
			break

# main ---------------------------------------------------------------------------------------------------

num_processes = int(sys.argv[1])
seed = int(sys.argv[2])											#48-bit linear congruential generator
lamb = float(sys.argv[3])										#exp-random.c
upper_bound = int(sys.argv[4])
cs_time = int(sys.argv[5])										#context switch time
alpha = float(sys.argv[6])										#estimate for SJF and SRT
slice_time = int(sys.argv[7])									#time slice value for RR
rr_add = sys.argv[8] if len(sys.argv) > 8 else "END"			#adding format for RR

random.srand48(seed)

processes = [None] * num_processes
temp_seed = seed

for i in range(num_processes):
	processes[i] = Process(i+1)
	processes[i].make_bursts(lamb, upper_bound)
	process_arrival(processes[i])

fcfs()

for i in range(num_processes):
	process_arrival(processes[i])

sjf(processes, cs_time, alpha)

for i in range(num_processes):
	process_arrival(processes[i])

srt()

for i in range(num_processes):
	process_arrival(processes[i])

rr(processes, slice_time, cs_time)
