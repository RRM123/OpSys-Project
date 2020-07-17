import sys
import math
import time
import random48 as rand

alphabet = {1: 'A', 2: 'B', 3: 'C', 4: 'D', 5: 'E', 6: 'F', 7: 'G', 8: 'H', 9: 'I', 10: 'J', 11: 'K', 12: 'L', 13: 'M', 14: 'N', 15: 'O', 16: 'P', 17: 'Q', 18: 'R', 19: 'S', 20: 'T', 21: 'U', 22: 'V', 23: 'W', 24: 'X', 25: 'Y', 26: 'Z'}
random = rand.Random()

class Process(object):
	def __init__(self, num, lamb, alpha):
		self.name = alphabet[num]
		self.init_arrival = 0
		self.num_bursts = 0
		self.current_burst = 0
		self.alpha = alpha
		self.tau = 1 / lamb
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

	def get_tau(self):
		return self.tau

	def change_cpu_time(self, burst, new_time):
		self.cpu_times[burst] = new_time

	def increment_burst(self):
		self.current_burst += 1

	def update_tau(self):
		self.tau = math.ceil((self.alpha * (self.cpu_times[self.current_burst - 1])) + ((1 - alpha) * self.tau))

	def reset_bursts(self):
		self.current_burst = 0

	def make_bursts(self, lamb, upper_bound):
		self.init_arrival = int(math.floor(random.get_random(lamb, upper_bound)))
		self.num_bursts = int(math.floor(random.drand48()*100) + 1)
		self.cpu_times = [0] * self.num_bursts
		self.io_times = [0] * self.num_bursts

		for i in range(self.num_bursts-1):
			self.cpu_times[i] = math.ceil(random.get_random(lamb, upper_bound))
			self.io_times[i] = math.ceil(random.get_random(lamb, upper_bound))

		self.cpu_times[self.num_bursts-1] = math.ceil(random.get_random(lamb, upper_bound))
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

	def get_complete_io_processes(self, timer):
		return [key for key in self.io.keys() if self.io[key] == timer]

	def queue_size(self):
		return len(self.queue)

	def addProcessToQueue(self, process):
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
		self.io.pop(process)

	def print_queue(self):
		print("[Q", end = " ")
		if len(self.queue) == 0:
			print("<empty>]")
		else:
			for i in range(len(self.queue) - 1):
				print(self.queue[i].get_name(), end = " ")
			print(self.queue[len(self.queue)-1].get_name() + "]")

	def sortSJFHelper(process):
		return process.get_cpu_io_times(process.get_current_burst())[0]

	def sortQueueSJF(self):
		self.queue = sorted(self.queue, key = self.sortSJFHelper())

	def reset(self):
		self.queue = []
		self.cpu = None
		self.io = {}

# simulations --------------------------------------------------------------------------------------------

# print functions
def printArrival(timer, name, tau = 0, isSJF = False):
	if (isSJF):
		print("time " + str(int(timer)) + "ms: " + "Process " + name + " (tau " + str(int(tau)) + "ms) arrived; added to ready queue", end = " ")
	else:
		print("time " + str(int(timer)) + "ms: " + "Process " + name + " arrived; added to ready queue", end = " ")

def printCPUStart(timer, name, cpu_time, tau = 0, isSJF = False):
	if (isSJF):
		print("time " + str(int(timer)) + "ms: " + "Process " + name + " (tau " + str(int(tau)) + "ms) started using the CPU for " + str(int(cpu_time)) + "ms burst", end = " ")
	else:
		print("time " + str(int(timer)) + "ms: " + "Process " + name + " started using the CPU for " + str(int(cpu_time)) + "ms burst", end = " ")

def printCPUEnd(timer, name, burst_num, tau = 0, isSJF = False):
	if (isSJF):
		print("time " + str(int(timer)) + "ms: " + "Process " + name + " (tau " + str(int(tau)) + "ms) completed a CPU burst; " + str(int(burst_num)) + " bursts to go", end = " ")
	else:
		print("time " + str(int(timer)) + "ms: " + "Process " + name + " completed a CPU burst; " + str(int(burst_num)) + " bursts to go", end = " ")

def printSwitchToIO(timer, name, io_time):
	print("time " + str(int(timer)) + "ms: " + "Process " + name + " switching out of CPU; will block on I/O until time " + str(int(io_time)) + "ms", end = " ")

def printIOComplete(timer, name, tau = 0, isSJF = False):
	if (isSJF):
		print("time " + str(int(timer)) + "ms: " + "Process " + name + " (tau " + str(int(tau)) + "ms) completed I/O; added to ready queue", end = " ")
	else:
		print("time " + str(int(timer)) + "ms: " + "Process " + name + " completed I/O; added to ready queue", end = " ")

def printTermination(timer, name):
	print("time " + str(int(timer)) + "ms: " + "Process " + name + " terminated", end = " ")

def printPreemption(timer, name, cpu_time, tau = 0, isSJF = False):
	if (isSJF):
		print("time " + str(int(timer)) + "ms: " + "Time slice expired; process " + name + " preempted with " + str(int(cpu_time)) + "ms to go", end = " ")
	else:
		print("time " + str(int(timer)) + "ms: " + "Time slice expired; process " + name + " preempted with " + str(int(cpu_time)) + "ms to go", end = " ")

def process_arrival(process, tau = 0, isSJF = False):
	if (isSJF):
		print("Process " + str(process.get_name()) + " [NEW] (arrival time " + str(process.get_init_arrival()) + " ms) " + str(process.get_num_bursts()) + " CPU bursts (tau " + str(int(tau)) + "ms)")
	else:
		print("Process " + str(process.get_name()) + " [NEW] (arrival time " + str(process.get_init_arrival()) + " ms) " + str(process.get_num_bursts()) + " CPU bursts")
	#print process.arrival()

def printNewTau(timer, tau, process_name):
	print("time " + str(int(timer)) + "ms: Recalculated tau = " + str(int(tau)) + "ms for process " + process_name, end = " ")

# main simulation functions
def fcfs(temp_processes, cs_time):
	processes = sorted(temp_processes, key = sortByArrivalTime)
	print("time 0ms: " + "Simulator started for FCFS [Q <empty>]")

	fcfs_simulation = Simulation()

	current_arrival = 0
	current_cpu_process = fcfs_simulation.get_CPU_process()
	current_bursts = {}
	terminated_processes = {}
	for i in range(len(processes)):
		current_bursts[processes[i].get_name()] = 0
		terminated_processes[processes[i].get_name()] = False
	timer = 0
	cpu_start_time = -1
	cpu_available_time = 0
	complete_io_processes = []
	checked = False

	while True:

		# CPU process done, switch to I/O
		if current_cpu_process != None and cpu_start_time + current_cpu_process.get_cpu_io_times(current_bursts[current_cpu_process.get_name()])[0] == timer:
			fcfs_simulation.removeProcessFromCPU(current_cpu_process)
			num_bursts = current_cpu_process.get_num_bursts() - current_bursts[current_cpu_process.get_name()] - 1
			if num_bursts <= 0:
				printTermination(timer, current_cpu_process.get_name())
				fcfs_simulation.print_queue()
				terminated_processes[current_cpu_process.get_name()] = True
			else:
				if timer <= 999:
					printCPUEnd(timer, current_cpu_process.get_name(), num_bursts)
					fcfs_simulation.print_queue()
				fcfs_simulation.addProcessToIO(current_cpu_process, timer + current_cpu_process.get_cpu_io_times(current_bursts[current_cpu_process.get_name()])[1] + (cs_time/2))
				if timer <= 999:
					printSwitchToIO(timer, current_cpu_process.get_name(), fcfs_simulation.get_io_end_time(current_cpu_process))
					fcfs_simulation.print_queue()
			cpu_available_time = timer + (cs_time/2)
			current_bursts[current_cpu_process.get_name()] += 1
			current_cpu_process = fcfs_simulation.get_CPU_process()
			continue

		# add to CPU
		if fcfs_simulation.queue_size() > 0 and current_cpu_process == None:
			#timer += (cs_time/2)
			fcfs_simulation.addProcessToCPU(fcfs_simulation.get_next_process())
			cpu_start_time = max(cpu_available_time, timer) + (cs_time/2)
			current_cpu_process = fcfs_simulation.get_CPU_process()
			checked = False
			continue

		# print addition to CPU
		if timer == cpu_start_time and not checked and timer <= 999:
			checked = True
			printCPUStart(timer, current_cpu_process.get_name(), current_cpu_process.get_cpu_io_times(current_bursts[current_cpu_process.get_name()])[0])
			fcfs_simulation.print_queue()

		# add processes done with I/O to queue
		complete_io_processes = fcfs_simulation.get_complete_io_processes(timer)
		if len(complete_io_processes) > 0:
			for process in complete_io_processes:
				fcfs_simulation.removeProcessFromIO(process)
				fcfs_simulation.addProcessToQueue(process)
				if timer <= 999:
					printIOComplete(timer, process.get_name())
					fcfs_simulation.print_queue()
			continue

		# new arrival
		if current_arrival < len(processes) and processes[current_arrival].get_init_arrival() == timer:
			fcfs_simulation.addProcessToQueue(processes[current_arrival])
			if timer <= 999:
				printArrival(timer, processes[current_arrival].get_name())
				fcfs_simulation.print_queue()
			current_arrival += 1
			continue

		# all processes terminated
		if False not in terminated_processes.values():
			break


		timer += 1
		# testing
		# if timer > 29000:
		# 	break

	timer += 2
	print("time " + str(int(timer)) + "ms: " + "Simulator ended for FCFS [Q <empty>]")
	print("")

def resolveTie(sjf_queue):
	n = len(sjf_queue)
	for i in range(n):
		for j in range(0, n - i - 1):
			if sjf_queue[j].tau == sjf_queue[j+1].tau:
				if sjf_queue[j].get_name() > sjf_queue[j+1].get_name():
					sjf_queue[j], sjf_queue[j+1] = sjf_queue[j+1], sjf_queue[j]

def sortByCPUTime(process):
	return process.get_tau()

def sjf(temp_processes, cs_time, alpha):
	processes = sorted(temp_processes, key = sortByArrivalTime)

	sjf_simulation = Simulation()

	add_half_context = False
	current_arrival = 0
	current_cpu_process = sjf_simulation.get_CPU_process()
	timer = 0
	cpu_start_time = 0
	print("time 0ms: " + "Simulator started for SJF [Q <empty>]")

	while True:
		# Move from CPU to I/O
		if current_cpu_process != None and cpu_start_time + current_cpu_process.get_cpu_io_times(current_cpu_process.get_current_burst())[0] <= timer:
			add_half_context = True
			#timer += (cs_time/2)
			sjf_simulation.removeProcessFromCPU(current_cpu_process)
			if (current_cpu_process.get_current_burst() == current_cpu_process.get_num_bursts() - 1):
				printTermination(timer, current_cpu_process.get_name())
				sjf_simulation.print_queue()
			else:
				sjf_simulation.addProcessToIO(current_cpu_process, timer + current_cpu_process.get_cpu_io_times(current_cpu_process.get_current_burst())[1] + (cs_time/2))
				current_cpu_process.increment_burst()
				if (timer <= 999):
					printCPUEnd(timer, current_cpu_process.get_name(), current_cpu_process.get_num_bursts() - current_cpu_process.get_current_burst(), current_cpu_process.get_tau(), True)
					sjf_simulation.print_queue()
				current_cpu_process.update_tau()
				if (timer <= 999):
					printNewTau(timer, current_cpu_process.get_tau(), current_cpu_process.get_name())
					sjf_simulation.print_queue()
					printSwitchToIO(timer, current_cpu_process.get_name(), sjf_simulation.get_io_end_time(current_cpu_process))
					sjf_simulation.print_queue()
			current_cpu_process = sjf_simulation.get_CPU_process()

		# Move from I/O to Ready Queue
		complete_io_processes = sjf_simulation.get_complete_io_processes(timer)
		if len(complete_io_processes) > 0:
			for io_process in complete_io_processes:
				sjf_simulation.removeProcessFromIO(io_process)
				sjf_simulation.addProcessToQueue(io_process)
				sjf_simulation.queue = sorted(sjf_simulation.queue, key= sortByCPUTime)
				resolveTie(sjf_simulation.queue)
				if (timer <= 999):
					printIOComplete(timer, io_process.get_name(), io_process.get_tau(), True)
					sjf_simulation.print_queue()

		# Process Arrival
		if current_arrival < len(processes) and processes[current_arrival].get_init_arrival() <= timer:
			sjf_simulation.addProcessToQueue(processes[current_arrival])
			sjf_simulation.queue = sorted(sjf_simulation.queue, key = sortByCPUTime)
			resolveTie(sjf_simulation.queue)
			if (timer <= 999):
				printArrival(processes[current_arrival].get_init_arrival(), processes[current_arrival].get_name(), processes[current_arrival].get_tau(), True)
				sjf_simulation.print_queue()
			current_arrival += 1

		# Add to CPU
		if sjf_simulation.queue_size() > 0 and current_cpu_process == None:
			timer += (cs_time/2)
			sjf_simulation.addProcessToCPU(sjf_simulation.get_next_process())
			cpu_start_time = timer
			if add_half_context:
				cpu_start_time += (cs_time/2)
			#print("Starting at " + str(timer))
			current_cpu_process = sjf_simulation.get_CPU_process()
			if (timer <= 999):
				printCPUStart(cpu_start_time, current_cpu_process.get_name(), current_cpu_process.get_cpu_io_times(current_cpu_process.get_current_burst())[0], current_cpu_process.get_tau(), True)
				sjf_simulation.print_queue()

		if add_half_context:
			timer += (cs_time/2)
			add_half_context = False
		else:
			timer += 1
		# testing
		if timer > 48000:
			break
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
	terminated_processes = {}
	for i in range(len(processes)):
		current_bursts[processes[i].get_name()] = 0
		terminated_processes[processes[i].get_name()] = False
	timer = 0
	cpu_start_time = -1
	cpu_available_time = 0
	complete_io_processes = []
	checked = False

	while True:

		# CPU process done, switch to I/O
		if current_cpu_process != None and cpu_start_time + current_cpu_process.get_cpu_io_times(current_bursts[current_cpu_process.get_name()])[0] == timer:
			rr_simulation.removeProcessFromCPU(current_cpu_process)
			num_bursts = current_cpu_process.get_num_bursts() - current_bursts[current_cpu_process.get_name()] - 1
			if num_bursts <= 0:
				printTermination(timer, current_cpu_process.get_name())
				rr_simulation.print_queue()
				terminated_processes[current_cpu_process.get_name()] = True
			else:
				if timer <= 999:
					printCPUEnd(timer, current_cpu_process.get_name(), num_bursts)
					rr_simulation.print_queue()
				rr_simulation.addProcessToIO(current_cpu_process, timer + current_cpu_process.get_cpu_io_times(current_bursts[current_cpu_process.get_name()])[1] + (cs_time/2))
				if timer <= 999:
					printSwitchToIO(timer, current_cpu_process.get_name(), rr_simulation.get_io_end_time(current_cpu_process))
					rr_simulation.print_queue()
			cpu_available_time = timer + (cs_time/2)
			current_bursts[current_cpu_process.get_name()] += 1
			current_cpu_process = rr_simulation.get_CPU_process()
			continue

		# CPU process preempted, add to queue
		# if current_cpu_process != None and cpu_start_time + slice_time == timer:
		# 	if rr_simulation.queue_size() == 0:
		# 		print("time " + str(int(timer)) + "ms: " + "Time slice expired; no preemption because ready queue is empty", end = " ")
		# 		rr_simulation.print_queue()
		# 	else:
		# 		rr_simulation.removeProcessFromCPU(current_cpu_process)
		# 		new_time = current_cpu_process.get_cpu_io_times(current_bursts[current_cpu_process.get_name()])[0] - slice_time
		# 		current_cpu_process.change_cpu_time(current_bursts[current_cpu_process.get_name()], new_time)
		# 		printPreemption(timer, current_cpu_process.get_name(), new_time)
		# 		rr_simulation.print_queue()
		# 		rr_simulation.addProcessToQueue(current_cpu_process)
		# 		cpu_available_time = timer + (cs_time/2)
		# 		current_cpu_process = rr_simulation.get_CPU_process()
		# 	continue

		# add to CPU
		if rr_simulation.queue_size() > 0 and current_cpu_process == None:
			#timer += (cs_time/2)
			rr_simulation.addProcessToCPU(rr_simulation.get_next_process())
			cpu_start_time = max(cpu_available_time, timer) + (cs_time/2)
			current_cpu_process = rr_simulation.get_CPU_process()
			checked = False
			continue

		# print addition to CPU
		if timer == cpu_start_time and not checked and timer <= 999:
			checked = True
			printCPUStart(timer, current_cpu_process.get_name(), current_cpu_process.get_cpu_io_times(current_bursts[current_cpu_process.get_name()])[0])
			rr_simulation.print_queue()

		# add processes done with I/O to queue
		complete_io_processes = rr_simulation.get_complete_io_processes(timer)
		if len(complete_io_processes) > 0:
			for process in complete_io_processes:
				rr_simulation.removeProcessFromIO(process)
				rr_simulation.addProcessToQueue(process)
				if timer <= 999:
					printIOComplete(timer, process.get_name())
					rr_simulation.print_queue()
			continue

		# new arrival
		if current_arrival < len(processes) and processes[current_arrival].get_init_arrival() == timer:
			rr_simulation.addProcessToQueue(processes[current_arrival])
			if timer <= 999:
				printArrival(timer, processes[current_arrival].get_name())
				rr_simulation.print_queue()
			current_arrival += 1
			continue

		# all processes terminated
		if False not in terminated_processes.values():
			break

		timer += 1
		# testing
		if timer > 42000:
			break

	timer += 2
	print("time " + str(int(timer)) + "ms: " + "Simulator ended for RR [Q <empty>]")

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
	processes[i] = Process(i+1, lamb, alpha)
	processes[i].make_bursts(lamb, upper_bound)
	processes[i].reset_bursts()
	process_arrival(processes[i])
"""
fcfs(processes, cs_time)
"""
for i in range(num_processes):
	processes[i].reset_bursts()
	process_arrival(processes[i], processes[i].get_tau(), True)

sjf(processes, cs_time, alpha)
"""
for i in range(num_processes):
	processes[i].reset_bursts()
	process_arrival(processes[i])

srt()

for i in range(num_processes):
	processes[i].reset_bursts()
	process_arrival(processes[i])

rr(processes, slice_time, cs_time)

"""