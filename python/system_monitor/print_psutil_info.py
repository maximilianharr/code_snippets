#!/usr/bin/env python

#  @file print_psutil_info.py
#  @author Maximilian Harr <maximilian.harr@gmail.com>
#  @date 22.11.2017
#
#  @brief Testing function that uses psutils to get cpu/memory/disk usage of system
# 				and rosnodes launched in 'workspace' directory.
#  @note GPU usage has to be gathered from nvidia-smi
#
# 				psutil: Cross-platform lib for process and system monitoring in Python.
# 				https://pypi.python.org/pypi/psutil
# 				Documentation: http://psutil.readthedocs.io/en/latest/
#
#  @bug
#
#
#  @todo 
#
#

import psutil, time
import subprocess
import pprint
import sys

import monitor_config as mcfg

if len(sys.argv) == 2:
	pid_name = sys.argv[1]
else:
	pid_name = 'roscore'

while 1:
	print "\n### SYSTEM INFO ###"

	print "\n### ROS INFO ###"
	pids = psutil.pids()
	for proc in psutil.process_iter():
		# Check if PID-name is correct and print
		if proc.name().find(pid_name)!=-1:
			print proc.as_dict(attrs=['pid','name','cpu_times','memory_info'])
			print proc.cpu_percent(interval=None)
			dictionary = proc.as_dict(attrs=['pid','name','cpu_times','memory_info'])
			data = dictionary.get("memory_info", "") 
		# Get all ROS processes and track cpu usage
		try: 
			found_ros = proc.exe().find( mcfg.config['workspace'] )
		except psutil.AccessDenied:
			pass # do nothing and ignore
		finally:
			if found_ros!=-1:
				print proc.as_dict(attrs=['pid','name','cpu_times','memory_info'])
				print proc.cpu_percent(interval=None)

	# cpu_percent takes time
	cpu_percentages = psutil.cpu_percent(interval=None, percpu=True)
	
	print "\n### CPU INFO ###"
	print psutil.cpu_times() # See also /proc/stat
	print psutil.cpu_times_percent()
	print cpu_percentages

	print "\n### MEMORY INFO ###"
	print psutil.virtual_memory()
	print psutil.swap_memory()
 
	print "\n### STORAGE INFO ###"
	print psutil.disk_usage('/')
	print psutil.disk_io_counters(perdisk=False)

	time.sleep( mcfg.config['sleep'] )

