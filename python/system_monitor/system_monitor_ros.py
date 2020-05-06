
#!/usr/bin/env python

#  @file system_monitor_ros.py
#  @author Maximilian Harr <maximilian.harr@gmail.com>
#  @date 22.11.2017
#
#  @brief Uses nvidia-smi and psutils to get cpu/gpu/memory/disk usage and publish
#         As ROS topic
#
#
#
#  @bug
#
#
#  @todo 
#
#

import rospy
import psutil, time
import subprocess
import pprint
import sys
import matplotlib.pyplot as plt
import numpy as np

import std_msgs.msg
from opel_msgs.msg import SysProfile
from opel_msgs.msg import SysProfileRosnode

import parse_nvidia_smi as pns
import monitor_config as mcfg

def addRosPidProfiles(sys_profile):
  nvidia_smi_list = pns.getNvidiaSmiAsList()
  list_of_process_id = pns.parseProcesses(nvidia_smi_list)
  pids = psutil.pids()
  for proc in psutil.process_iter():
    try: 
      found_ros = proc.exe().find( mcfg.config['workspace'] )
    except psutil.AccessDenied:
      pass # do nothing and ignore
    finally:
      if found_ros!=-1:
        sys_profile_rosnode = SysProfileRosnode()
        # Read data from psutils and store in sys_profile_rosnode
        dictionary = proc.as_dict(attrs=['pid','name','cpu_times','memory_info'])
        cpu_percent = proc.cpu_percent(interval=None)
        sys_profile_rosnode.pid               = dictionary.get("pid", "")
        sys_profile_rosnode.memory_info_rss   = dictionary.get("memory_info", "").rss
        sys_profile_rosnode.memory_info_vms   = dictionary.get("memory_info", "").vms
        sys_profile_rosnode.cpu_times_user    = dictionary.get("cpu_times", "").user
        sys_profile_rosnode.cpu_times_system  = dictionary.get("cpu_times", "").system
        sys_profile_rosnode.cpu_percent       = cpu_percent
        sys_profile_rosnode.name              = dictionary.get("name", "")
        # Find process ID in nvidia_smi_list and store
        for i in range(len(list_of_process_id)):
          if int(list_of_process_id[i].Process_ID)==sys_profile_rosnode.pid:
            sys_profile_rosnode.type = list_of_process_id[i].Type
            sys_profile_rosnode.used_gpu_memory = float(list_of_process_id[i].Used_GPU_Memory)
        sys_profile.rosnodes.append(sys_profile_rosnode)
  return sys_profile

def addSystemProfile(sys_profile):
  # CPU
  cpu_times = psutil.cpu_times() 
  sys_profile.cpu.cpu_times.user        = cpu_times.user
  sys_profile.cpu.cpu_times.nice        = cpu_times.nice
  sys_profile.cpu.cpu_times.system      = cpu_times.system
  sys_profile.cpu.cpu_times.idle        = cpu_times.idle
  sys_profile.cpu.cpu_times.iowait      = cpu_times.iowait
  sys_profile.cpu.cpu_times.irq         = cpu_times.irq
  sys_profile.cpu.cpu_times.softirq     = cpu_times.softirq
  sys_profile.cpu.cpu_times.steal       = cpu_times.steal
  sys_profile.cpu.cpu_times.guest       = cpu_times.guest
  sys_profile.cpu.cpu_times.guest_nice  = cpu_times.guest_nice

  cpu_percent = psutil.cpu_percent(interval=None, percpu=True) # percpu: True: Array with all CPU percenteges | False: Mean percentage
  sys_profile.cpu.cpu_percent.value = cpu_percent

  # Disk
  disk_usage = psutil.disk_usage('/')
  sys_profile.disk.disk_usage.total   = disk_usage.total
  sys_profile.disk.disk_usage.used    = disk_usage.used
  sys_profile.disk.disk_usage.free    = disk_usage.free
  sys_profile.disk.disk_usage.percent = disk_usage.percent

  disk_io_counters = psutil.disk_io_counters(perdisk=False)
  sys_profile.disk.disk_io_counters.read_count  = disk_io_counters.read_count
  sys_profile.disk.disk_io_counters.write_count = disk_io_counters.write_count
  sys_profile.disk.disk_io_counters.read_bytes  = disk_io_counters.read_bytes
  sys_profile.disk.disk_io_counters.write_bytes = disk_io_counters.write_bytes
  
  # GPU
  nvidia_smi_list = pns.getNvidiaSmiAsList()
  
  gpu_utilization = pns.parseUtilization(nvidia_smi_list)
  sys_profile.gpu.Utilization_Gpu     = int(gpu_utilization.Gpu)
  sys_profile.gpu.Utilization_Memory  = int(gpu_utilization.Memory)
  sys_profile.gpu.Utilization_Encoder = int(gpu_utilization.Encoder)
  sys_profile.gpu.Utilization_Decoder = int(gpu_utilization.Decoder)

  gpu_temperature = pns.parseTemperature(nvidia_smi_list)
  sys_profile.gpu.Temperature_GPU_Current_Temp  = int(gpu_temperature.GPU_Current_Temp)
  sys_profile.gpu.Temperature_GPU_Shutdown_Temp = int(gpu_temperature.GPU_Shutdown_Temp)
  sys_profile.gpu.Temperature_GPU_Slowdown_Temp = int(gpu_temperature.GPU_Slowdown_Temp)

  gpu_fb_memory_usage = pns.parseFB_Memory_Usage(nvidia_smi_list)
  sys_profile.gpu.FB_Memory_Usage_Total = int(gpu_fb_memory_usage.Total)
  sys_profile.gpu.FB_Memory_Usage_Used  = int(gpu_fb_memory_usage.Used)
  sys_profile.gpu.FB_Memory_Usage_Free  = int(gpu_fb_memory_usage.Free)

  gpu_bar1_memory_usage = pns.parseBAR1_Memory_Usage(nvidia_smi_list)
  sys_profile.gpu.BAR1_Memory_Usage_Total = int(gpu_bar1_memory_usage.Total)
  sys_profile.gpu.BAR1_Memory_Usage_Used  = int(gpu_bar1_memory_usage.Used)
  sys_profile.gpu.BAR1_Memory_Usage_Free  = int(gpu_bar1_memory_usage.Free)

  # Memory
  virtual_memory = psutil.virtual_memory()
  sys_profile.memory.virtual_memory.total     = virtual_memory.total
  sys_profile.memory.virtual_memory.available = virtual_memory.available
  sys_profile.memory.virtual_memory.percent   = virtual_memory.percent
  sys_profile.memory.virtual_memory.used      = virtual_memory.used
  sys_profile.memory.virtual_memory.free      = virtual_memory.free
  sys_profile.memory.virtual_memory.active    = virtual_memory.active
  sys_profile.memory.virtual_memory.inactive  = virtual_memory.inactive
  sys_profile.memory.virtual_memory.buffers   = virtual_memory.buffers
  sys_profile.memory.virtual_memory.cached    = virtual_memory.cached

  swap_memory = psutil.swap_memory()
  sys_profile.memory.swap_memory.total   = swap_memory.total
  sys_profile.memory.swap_memory.used    = swap_memory.used
  sys_profile.memory.swap_memory.free    = swap_memory.free
  sys_profile.memory.swap_memory.percent = swap_memory.percent
  sys_profile.memory.swap_memory.sin     = swap_memory.sin
  sys_profile.memory.swap_memory.sout    = swap_memory.sout

  return sys_profile

def readSysProfile():
  sys_profile = SysProfile()
  sys_profile.header = std_msgs.msg.Header()
  sys_profile.header.stamp = rospy.Time.now()
  sys_profile = addRosPidProfiles(sys_profile)
  sys_profile = addSystemProfile(sys_profile)
  return sys_profile

def publishSysProfile(sys_profile):
  #print sys_profile
  pub = rospy.Publisher(mcfg.config['topic'], SysProfile, queue_size=10)
  pub.publish(sys_profile)

if __name__ == '__main__':
  rospy.init_node('system_monitor_ros', anonymous=True)
  rate = rospy.Rate(10)
  # Create bar plot
  fig, ax = plt.subplots()
  width = 0.5
  try:
    while not rospy.is_shutdown():
      sys_profile = readSysProfile()
      publishSysProfile(sys_profile)
      # Print CPU Usage of processes to bar chart
      labels = []
      values = []
      if len(sys_profile.rosnodes)!=0 and mcfg.config['barplot']:
        for i in range( len(sys_profile.rosnodes)):
          labels.append(sys_profile.rosnodes[i].name)
          values.append(sys_profile.rosnodes[i].cpu_percent)
    
        ind = np.arange(len(values))
        y_pos = np.arange(len(labels))
        ax.barh(ind, values, color='blue')
        ax.set_yticks(ind+width/2)
        ax.set_yticklabels(labels, minor=False)
        ax.set_xlabel('Percent %')
        ax.set_title('CPU Usage')
        for i, v in enumerate(values):
          ax.text(v, i + .25, str(v)+' %', color='blue', fontweight='bold')
        plt.pause(0.01)
        plt.draw()
        ax.clear()
      time.sleep( mcfg.config['sleep'] )
  except rospy.ROSInterruptException:
    # ... some last commands before the node terminates
    pass
