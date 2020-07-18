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
		self.lamb = lamb
		self.tau = 1 / lamb
		self.cpu_times = []
		self.remaining_cpu = []
		self.io_times = []
		self.preempted = False

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

	def wasPreempted(self):
		return self.preempted

	def change_cpu_time(self, burst, new_time):
		self.cpu_times[burst] = new_time

	def set_preempted(self, val):
		self.preempted = val

	def increment_burst(self):
		self.current_burst += 1

	def update_tau(self):
		self.tau = math.ceil((self.alpha * (self.cpu_times[self.current_burst - 1])) + ((1 - alpha) * self.tau))

	def reset_bursts(self):
		self.tau = 1/self.lamb
		self.current_burst = 0

	def make_bursts(self, lamb, upper_bound):
		self.init_arrival = int(math.floor(random.get_random(lamb, upper_bound)))
		self.num_bursts = int(math.floor(random.drand48()*100) + 1)
		self.cpu_times = [0] * self.num_bursts
		self.remaining_cpu = [0] * self.num_bursts
		self.io_times = [0] * self.num_bursts

		for i in range(self.num_bursts-1):
			self.cpu_times[i] = math.ceil(random.get_random(lamb, upper_bound))
			self.remaining_cpu[i] = self.cpu_times[i]
			self.io_times[i] = math.ceil(random.get_random(lamb, upper_bound))

		self.cpu_times[self.num_bursts-1] = math.ceil(random.get_random(lamb, upper_bound))
		self.remaining_cpu[self.num_bursts-1] = self.cpu_times[self.num_bursts-1]
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

	def addProcessToQueue(self, process, beginning = False):
		# wait for wait_time
		# start_time = time.time()
		# while int(time.time() - start_time)*1000 < wait_time:
		# 	continue
		if beginning:
			self.queue.insert(0, process)
		else:
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

def printCPURemaining(timer, name, cpu_time, tau = 0, isSJF = False):
	if (isSJF):
		print("time " + str(int(timer)) + "ms: " + "Process " + name + " (tau " + str(int(tau)) + "ms) started using the CPU with " + str(int(cpu_time)) + "ms burst remaining", end = " ")
	else:
		print("time " + str(int(timer)) + "ms: " + "Process " + name + " started using the CPU with " + str(int(cpu_time)) + "ms burst remaining", end = " ")

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
	num_cs = 0

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
			num_cs += 1
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

	return num_cs

def resolveTie(sjf_queue):
	n = len(sjf_queue)
	for i in range(n):
		for j in range(0, n - i - 1):
			if sjf_queue[j].tau == sjf_queue[j+1].tau:
				if sjf_queue[j].get_name() > sjf_queue[j+1].get_name():
					sjf_queue[j], sjf_queue[j+1] = sjf_queue[j+1], sjf_queue[j]

def sortByCPUTime(process):
	return process.get_tau()

def sjf(temp_processes, cs_time):
	processes = sorted(temp_processes, key = sortByArrivalTime)

	sjf_simulation = Simulation()
	first_process_added = False
	add_half_context = False
	current_arrival = 0
	current_cpu_process = sjf_simulation.get_CPU_process()
	timer = 0
	cpu_start_time = -1
	cpu_available_time = 0
	checked = False
	num_cs = 0

	print("time 0ms: " + "Simulator started for SJF [Q <empty>]")

	while True:
		# Move from CPU to I/O
		if current_cpu_process != None and cpu_start_time + current_cpu_process.get_cpu_io_times(current_cpu_process.get_current_burst())[0] == timer:
			add_half_context = True
			#timer += (cs_time/2)
			sjf_simulation.removeProcessFromCPU(current_cpu_process)
			if (current_cpu_process.get_current_burst() == current_cpu_process.get_num_bursts() - 1):
				printTermination(timer, current_cpu_process.get_name())
				sjf_simulation.print_queue()
			else:
				sjf_simulation.addProcessToIO(current_cpu_process, timer + current_cpu_process.get_cpu_io_times(current_cpu_process.get_current_burst())[1] + (cs_time/2))
				#print(current_cpu_process.get_cpu_io_times(current_cpu_process.get_current_burst())[1] )
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
			cpu_available_time = timer + (cs_time/2)
			current_cpu_process = sjf_simulation.get_CPU_process()
			continue
		
		# Add to CPU
		if cpu_available_time <= timer and current_cpu_process == None and sjf_simulation.queue_size() > 0:
			#timer += (cs_time/2)
			num_cs += 1
			sjf_simulation.addProcessToCPU(sjf_simulation.get_next_process())
			cpu_start_time = max(cpu_available_time, timer) + (cs_time/2)
			#print("Starting at " + str(timer))
			current_cpu_process = sjf_simulation.get_CPU_process()
			checked = False
			continue

		if timer == cpu_start_time and not checked:
			checked = True
			if timer <= 999:
				printCPUStart(cpu_start_time, current_cpu_process.get_name(), current_cpu_process.get_cpu_io_times(current_cpu_process.get_current_burst())[0], current_cpu_process.get_tau(), True)
				sjf_simulation.print_queue()

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
			continue

		# Process Arrival
		if current_arrival < len(processes) and processes[current_arrival].get_init_arrival() <= timer:
			first_process_added = True
			sjf_simulation.addProcessToQueue(processes[current_arrival])
			sjf_simulation.queue = sorted(sjf_simulation.queue, key = sortByCPUTime)
			resolveTie(sjf_simulation.queue)
			if (timer <= 999):
				printArrival(processes[current_arrival].get_init_arrival(), processes[current_arrival].get_name(), processes[current_arrival].get_tau(), True)
				sjf_simulation.print_queue()
			current_arrival += 1
			continue
		
		# testing
		if current_cpu_process == None and len(sjf_simulation.io) == 0 and len(sjf_simulation.queue) == 0 and first_process_added:
			break
		timer += 1
	timer += 2
	print("time " + str(int(timer)) + "ms: " + "Simulator ended for SJF [Q <empty>]")
	print("")

	return num_cs

def srt(temp_processes, cs_time):
	processes = sorted(temp_processes, key = sortByArrivalTime)

	srt_simulation = Simulation()

	first_process_added = False
	current_arrival = 0
	current_cpu_process = srt_simulation.get_CPU_process()
	timer = 0
	cpu_start_time = -1
	cpu_available_time = 0
	checked = False
	cpu_removed = True
	cpu_remove_time = 0
	cpu_add_time = 0
	block_cpu_removal = False
	block_cpu_addition = False
	preempt_process = None
	num_cs = 0
	num_preemptions = 0

	print("time 0ms: " + "Simulator started for SRT [Q <empty>]")
	
	while True:
		# Move from CPU to I/O
		if current_cpu_process != None and cpu_start_time + current_cpu_process.remaining_cpu[current_cpu_process.current_burst] == timer and not block_cpu_removal:
			add_half_context = True
			#timer += (cs_time/2)
			srt_simulation.removeProcessFromCPU(current_cpu_process)
			if (current_cpu_process.get_current_burst() == current_cpu_process.get_num_bursts() - 1):
				printTermination(timer, current_cpu_process.get_name())
				srt_simulation.print_queue()
			else:
				srt_simulation.addProcessToIO(current_cpu_process, timer + current_cpu_process.get_cpu_io_times(current_cpu_process.get_current_burst())[1] + (cs_time/2))
				#print(current_cpu_process.get_cpu_io_times(current_cpu_process.get_current_burst())[1] )
				current_cpu_process.increment_burst()
				if (timer <= 999):
					printCPUEnd(timer, current_cpu_process.get_name(), current_cpu_process.get_num_bursts() - current_cpu_process.get_current_burst(), current_cpu_process.get_tau(), True)
					srt_simulation.print_queue()
				current_cpu_process.update_tau()
				if (timer <= 999):
					printNewTau(timer, current_cpu_process.get_tau(), current_cpu_process.get_name())
					srt_simulation.print_queue()
					printSwitchToIO(timer, current_cpu_process.get_name(), srt_simulation.get_io_end_time(current_cpu_process))
					srt_simulation.print_queue()
			cpu_available_time = timer + (cs_time/2)
			current_cpu_process = srt_simulation.get_CPU_process()
			continue
		
		# Move from I/O to Ready Queue
		complete_io_processes = srt_simulation.get_complete_io_processes(timer)
		if len(complete_io_processes) > 0:
			for io_process in complete_io_processes:
				srt_simulation.removeProcessFromIO(io_process)
				srt_simulation.addProcessToQueue(io_process)
				srt_simulation.queue = sorted(srt_simulation.queue, key= sortByCPUTime)
				resolveTie(srt_simulation.queue)
				if timer <= 999 and current_cpu_process != None and io_process.tau < (current_cpu_process.tau - (timer - cpu_start_time)) and preempt_process == None:
					print("time " + str(int(timer)) + "ms: " + "Process " + io_process.name + " (tau " + str(int(io_process.tau)) + "ms) completed I/O; preempting " + current_cpu_process.name, end = " ")
					srt_simulation.print_queue()
					preempt_process = io_process
					cpu_remove_time = timer + (cs_time/2)						# block removal of cpu until cs_time/2
					block_cpu_removal = True
					current_cpu_process.remaining_cpu[current_cpu_process.current_burst] -= (timer - cpu_start_time)
					# FIRST IF STATEMENT:
					# after cs_time/2, remove cpu_process ---- when cpu_remove_time == timer and block_cpu_removal
					# remove preempt_process from queue and change block_cpu_removal to False
					# block adding of cpu until cs_time/2 ---- assign cpu_add_time = timer + cs_time/2 and block_cpu_addition = True
					# SECOND IF STATEMENT:
					# after cs_time/2, add preempt_process ---- when cpu_add_time == timer and block_cpu_addition
					# cpu_start_time = timer
					# current_cpu_process = queue.pop
					# change block_cpu_addition to False
					# change preempt_process to None
				elif timer <= 999:
					printIOComplete(timer, io_process.get_name(), io_process.get_tau(), True)
					srt_simulation.print_queue()
			continue

		if cpu_remove_time == timer and block_cpu_removal and preempt_process != None:
			srt_simulation.removeProcessFromCPU(current_cpu_process)
			srt_simulation.addProcessToQueue(current_cpu_process)
			srt_simulation.queue = sorted(srt_simulation.queue, key= sortByCPUTime)
			resolveTie(srt_simulation.queue)
			current_cpu_process = None
			preempt_process = srt_simulation.get_next_process()
			block_cpu_removal = False
			cpu_add_time = timer + (cs_time/2)
			block_cpu_addition = True
			continue

		if cpu_add_time == timer and block_cpu_addition and preempt_process != None:
			srt_simulation.addProcessToCPU(preempt_process)
			cpu_start_time = timer
			current_cpu_process = preempt_process
			block_cpu_addition = False
			preempt_process = None
			print("time "+ str(timer) + "ms: Process " + current_cpu_process.name + " (tau "+ str(int(current_cpu_process.tau)) +"ms) started using the CPU with "+ str(current_cpu_process.remaining_cpu[current_cpu_process.current_burst]) +"ms burst remaining", end = " ")
			srt_simulation.print_queue()

		# Add to CPU
		if cpu_available_time <= timer and current_cpu_process == None and srt_simulation.queue_size() > 0 and not block_cpu_addition:
			#timer += (cs_time/2)
			num_cs += 1
			srt_simulation.addProcessToCPU(srt_simulation.get_next_process())
			cpu_start_time = max(cpu_available_time, timer) + (cs_time/2)
			#print("Starting at " + str(timer))
			current_cpu_process = srt_simulation.get_CPU_process()
			checked = False
			continue

		if timer == cpu_start_time and not checked and not block_cpu_addition:
			checked = True
			if timer <= 999:
				print("time "+ str(timer) + "ms: Process " + current_cpu_process.name + " (tau "+ str(int(current_cpu_process.tau)) +"ms) started using the CPU with "+ str(current_cpu_process.remaining_cpu[current_cpu_process.current_burst]) +"ms burst remaining", end = " ")
				srt_simulation.print_queue()

		# Process Arrival
		if current_arrival < len(processes) and processes[current_arrival].get_init_arrival() <= timer:
			first_process_added = True
			srt_simulation.addProcessToQueue(processes[current_arrival])
			srt_simulation.queue = sorted(srt_simulation.queue, key = sortByCPUTime)
			resolveTie(srt_simulation.queue)
			if (timer <= 999):
				printArrival(processes[current_arrival].get_init_arrival(), processes[current_arrival].get_name(), processes[current_arrival].get_tau(), True)
				srt_simulation.print_queue()
			current_arrival += 1
			continue
		
		# testing
		if current_cpu_process == None and len(srt_simulation.io) == 0 and len(srt_simulation.queue) == 0 and first_process_added:
			break
		timer += 1
	timer += 2
	print("time " + str(int(timer)) + "ms: " + "Simulator ended for SRT [Q <empty>]")
	print("")

	return num_cs, num_preemptions

def sortByArrivalTime(process):
	return process.get_init_arrival()

def sortByName(process):
	return process.get_name()


def rr(temp_processes, slice_time, cs_time, beginning):
	processes = sorted(temp_processes, key = sortByArrivalTime)
	print("time 0ms: " + "Simulator started for RR [Q <empty>]")

	rr_simulation = Simulation()

	current_arrival = 0
	current_cpu_process = rr_simulation.get_CPU_process()
	preempted_cpu_process = None
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
	num_cs = 0
	num_preemptions = 0

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
		if current_cpu_process != None and cpu_start_time + slice_time == timer:
			if rr_simulation.queue_size() == 0:
				if timer <= 999:
					print("time " + str(int(timer)) + "ms: " + "Time slice expired; no preemption because ready queue is empty", end = " ")
					rr_simulation.print_queue()
				cpu_start_time = timer
				new_time = current_cpu_process.get_cpu_io_times(current_bursts[current_cpu_process.get_name()])[0] - slice_time
				current_cpu_process.change_cpu_time(current_bursts[current_cpu_process.get_name()], new_time)
			else:
				rr_simulation.removeProcessFromCPU(current_cpu_process)
				current_cpu_process.set_preempted(True)
				num_preemptions += 1
				new_time = current_cpu_process.get_cpu_io_times(current_bursts[current_cpu_process.get_name()])[0] - slice_time
				current_cpu_process.change_cpu_time(current_bursts[current_cpu_process.get_name()], new_time)
				if timer <= 999:
					printPreemption(timer, current_cpu_process.get_name(), new_time)
					rr_simulation.print_queue()
				cpu_available_time = timer + (cs_time/2)
				preempted_cpu_process = current_cpu_process
				current_cpu_process = rr_simulation.get_CPU_process()
			continue

		# add preempted process to queue
		if preempted_cpu_process != None and timer == cpu_available_time:
			rr_simulation.addProcessToQueue(preempted_cpu_process)
			preempted_cpu_process = None
			continue

		# add to CPU
		if rr_simulation.queue_size() > 0 and current_cpu_process == None and timer >= cpu_available_time:
			#timer += (cs_time/2)
			num_cs += 1
			rr_simulation.addProcessToCPU(rr_simulation.get_next_process())
			cpu_start_time = timer + (cs_time/2)
			current_cpu_process = rr_simulation.get_CPU_process()
			checked = False
			continue

		# print addition to CPU
		if timer == cpu_start_time and not checked and timer <= 999:
			checked = True
			if current_cpu_process.wasPreempted():
				printCPURemaining(timer, current_cpu_process.get_name(), current_cpu_process.get_cpu_io_times(current_bursts[current_cpu_process.get_name()])[0])
				current_cpu_process.set_preempted(False)
			else:
				printCPUStart(timer, current_cpu_process.get_name(), current_cpu_process.get_cpu_io_times(current_bursts[current_cpu_process.get_name()])[0])
			rr_simulation.print_queue()

		# add processes done with I/O to queue
		temp_complete_io_processes = rr_simulation.get_complete_io_processes(timer)
		complete_io_processes = sorted(temp_complete_io_processes, key = sortByName)
		if len(complete_io_processes) > 0:
			for process in complete_io_processes:
				rr_simulation.removeProcessFromIO(process)
				rr_simulation.addProcessToQueue(process, beginning)
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
		# if timer > 42000:
		# 	break

	timer += 2
	print("time " + str(int(timer)) + "ms: " + "Simulator ended for RR [Q <empty>]")
	return num_cs, num_preemptions

def getAvgCPUBurstTime(processes):
	ans = 0
	size = 0
	for process in processes:
		size += process.get_num_bursts()
		for num in range(process.get_num_bursts()):
			ans += process.get_cpu_io_times(num)[0]

	return float(ans) / float(size)

# main ---------------------------------------------------------------------------------------------------

num_processes = int(sys.argv[1])
seed = int(sys.argv[2])											#48-bit linear congruential generator
lamb = float(sys.argv[3])										#exp-random.c
upper_bound = int(sys.argv[4])
cs_time = int(sys.argv[5])										#context switch time
alpha = float(sys.argv[6])										#estimate for SJF and SRT
slice_time = int(sys.argv[7])									#time slice value for RR
rr_add = sys.argv[8] if len(sys.argv) > 8 else "END"			#adding format for RR

add_beginning = True if rr_add == "BEGINNING" else False
random.srand48(seed)

processes = [None] * num_processes
temp_seed = seed

data_file = open("simout.txt", "w")

for i in range(num_processes):
	processes[i] = Process(i+1, lamb, alpha)
	processes[i].make_bursts(lamb, upper_bound)

avg_cpu_time = getAvgCPUBurstTime(processes)

for i in range(num_processes):
	processes[i].reset_bursts()
	process_arrival(processes[i])
"""
data_file.write("Algorithm FCFS\n")
data_file.write("-- average CPU burst time: " + str(round(avg_cpu_time, 3)) + " ms\n")
num_cs = fcfs(processes, cs_time)
data_file.write("-- total number of context switches: " + str(num_cs) + "\n")
data_file.write("-- total number of preemptions: 0\n")

for i in range(num_processes):
	processes[i].reset_bursts()
	process_arrival(processes[i], processes[i].get_tau(), True)

data_file.write("Algorithm SJF\n")
data_file.write("-- average CPU burst time: " + str(round(avg_cpu_time, 3)) + " ms\n")
num_cs = sjf(processes, cs_time)
data_file.write("-- total number of context switches: " + str(num_cs) + "\n")
data_file.write("-- total number of preemptions: 0\n")
"""
for i in range(num_processes):
	processes[i].reset_bursts()
	process_arrival(processes[i])

data_file.write("Algorithm SRT\n")
data_file.write("-- average CPU burst time: " + str(round(avg_cpu_time, 3)) + " ms\n")
num_cs, num_preemptions = srt(processes, cs_time)
data_file.write("-- total number of context switches: " + str(num_cs) + "\n")
data_file.write("-- total number of preemptions: " + str(num_preemptions) + "\n")
"""
for i in range(num_processes):
	processes[i].reset_bursts()
	process_arrival(processes[i])

data_file.write("Algorithm RR\n")
data_file.write("-- average CPU burst time: " + str(round(avg_cpu_time, 3)) + " ms\n")
num_cs, num_preemptions = rr(processes, slice_time, cs_time, add_beginning)
data_file.write("-- total number of context switches: " + str(num_cs) + "\n")
data_file.write("-- total number of preemptions: " + str(num_preemptions) + "\n")
"""
data_file.close()
