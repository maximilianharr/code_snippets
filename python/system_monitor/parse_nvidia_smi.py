#!/usr/bin/env python

#  @file parse_nvidia_smi.py
#  @author Maximilian Harr <maximilian.harr@gmail.com>
#  @date 22.11.2017
#
#  @brief Functions to get and parse nvidia-smi stream to namedtuples
#
#
#
#  @bug
#
#
#  @todo 
#
#


import collections # NamedTupels in Python: https://docs.python.org/2/library/collections.html
import subprocess
import pprint
import string

# Create names-tuples for nvida-smi output
Attached_GPU = collections.namedtuple('Attached_GPU', 'Product_Name') 
Utilization = collections.namedtuple('Utilization', 'Gpu Memory Encoder Decoder')
Process_ID = collections.namedtuple('Process_ID', 'Process_ID Type Name Used_GPU_Memory')
Temperature = collections.namedtuple('Temperature', 'GPU_Current_Temp GPU_Shutdown_Temp GPU_Slowdown_Temp')
FB_Memory_Usage = collections.namedtuple('FB_Memory_Usage', 'Total Used Free')
BAR1_Memory_Usage = collections.namedtuple('BAR1_Memory_Usage', 'Total Used Free')

# @brief Get indent of line
def getIndentOfLine(line):
  leading_spaces = len(line) - len(line.lstrip())
  return leading_spaces

# @brief Get block of list with increasing indents and 
def getBlockWithIncreasingIndent(line_string, line_list, skip_elem_n):
  skip_elem_count=-1
  line_string = string.replace(line_string, '_', ' ') # Remove underline
  list_iter = iter(line_list)
  # Iterate throug list
  for index,item in enumerate(list_iter):
    try:
      if item.find(line_string)!=-1:
        skip_elem_count+=1
        if skip_elem_count==skip_elem_n:
          first_indent = getIndentOfLine(item)
          count_line=1
          while getIndentOfLine( line_list[index+count_line]) > first_indent:
            count_line+=1
          cropped_list=line_list[index:(index+count_line)]
          return cropped_list
    except IndexError:
      print "Indexerror: End of line_list. Abort."
      break

def parseGeneric(namedtuple_obj, list_cropped):
  if list_cropped is None:
    return namedtuple_obj
  fields_ = namedtuple_obj._fields # Get field names of Utilization NamedTuple
  for field_name in fields_:
    field_name = string.replace(field_name, '_', ' ') # Remove underline
    list_iter = iter(list_cropped) # Reset iterator
    for item in list_iter:
      try:
        if item.find(field_name)!=-1:
          key, val = item.split(':')
          key, val = key.strip(), val.strip() # Removes empy spaces
          val = val.split(' ') # Divide value and unit
          field_name = string.replace(field_name, ' ', '_') # Add underline again
          namedtuple_obj = namedtuple_obj._replace(**{field_name:val[0]})
        #print getIndentOfLine(item),':',key,':',val
        # print item
      except:
        pass
  return namedtuple_obj

def countStringInList(line_string, line_list):
  count_lines=0
  list_iter = iter(line_list)
  # Iterate throug list
  for item in list_iter:
    if item.find(line_string)!=-1:
      count_lines+=1
  return count_lines
        

def parseUtilization(line_list):
  list_cropped_ = getBlockWithIncreasingIndent('Utilization', line_list, 0)
  obj_ = Utilization(-1,-1,-1,-1) # Init values to be -1  
  return parseGeneric(obj_, list_cropped_)

def parseProcesses(line_list):
  count_processes = countStringInList('Process ID', line_list)
  list_of_process_id = []
  for i in range(count_processes):
    list_cropped_ = getBlockWithIncreasingIndent('Process_ID', line_list, i)
    obj_ = Process_ID(-1,-1,-1,-1) # Init values to be -1  
    obj_ = parseGeneric(obj_, list_cropped_)
    list_of_process_id.append( obj_ )
  return list_of_process_id

def parseTemperature(line_list):
  list_cropped_ = getBlockWithIncreasingIndent('Temperature', line_list, 0)
  obj_ = Temperature(-1,-1,-1) # Init values to be -1  
  return parseGeneric(obj_, list_cropped_)

def parseFB_Memory_Usage(line_list):
  list_cropped_ = getBlockWithIncreasingIndent('FB_Memory_Usage', line_list, 0)
  obj_ = FB_Memory_Usage(-1,-1,-1) # Init values to be -1  
  return parseGeneric(obj_, list_cropped_)

def parseBAR1_Memory_Usage(line_list):
  list_cropped_ = getBlockWithIncreasingIndent('BAR1_Memory_Usage', line_list, 0)
  obj_ = BAR1_Memory_Usage(-1,-1,-1) # Init values to be -1  
  return parseGeneric(obj_, list_cropped_)

def getNvidiaSmiAsList():
  sp = subprocess.Popen(['nvidia-smi', '-q'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
  out_str = sp.communicate()
  out_list = out_str[0].split('\n')
  return out_list