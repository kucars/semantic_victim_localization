#!/usr/bin/env python

import glob
import os
from os.path import expanduser
import sys
import rospkg
import subprocess
import shutil

import rospkg
rospack  = rospkg.RosPack()
pkg_path = rospack.get_path('victim_localization')
resultFolder = pkg_path + '/Data'

home = expanduser("~")
roslogFolder = home +'/.ros/log'

def clean_dir(location):
    fileList = os.listdir(location)


def killProcessByName(scriptName):
  process = subprocess.Popen(["ps", "-eo","pid,command"], stdout=subprocess.PIPE)
  output = process.communicate()[0]
  splitted = output.rsplit('\n')
  for line in splitted:
    if scriptName in line:
      #print line
      l = line.lstrip()
      pid = l.split(' ')[0]
      #print pid
      os.system('kill -9 ' + pid)
filenames = []
# Get package path
rospack = rospkg.RosPack()
batchfolder = os.path.join(rospack.get_path('victim_localization'), 'batch')

# Get files in batch folder



os.chdir( batchfolder )
for file in sorted( glob.glob("*.yaml") ):
  filenames.append( file )
  print(file)

for file in filenames:
  filepath = os.path.join(batchfolder,file)
  file_no_ext = os.path.splitext(file)[0]

  resultfolder = os.path.join(batchfolder, file_no_ext)
  if not os.path.exists(resultfolder):
    os.makedirs(resultfolder)

    proc = subprocess.Popen(['rosrun victim_localization plot_ieration_info_combined.py' + file_no_ext],stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)

  # Run NBV
  os.system("roslaunch victim_localization nbv_test.launch debug:=false batch:=true param_file:=" + filepath)

  killProcessByName("plot_ieration_info_combined.py")
  # move all generated results into folder for later analysis
  # also delete ros log to avoid been accumulated
  for the_file in os.listdir(resultFolder):
      file_path = os.path.join(resultFolder, the_file)
      try:
          if os.path.isfile(file_path):
              shutil.move(file_path,resultfolder)
      except Exception as e:
          print(e)

  for the_file in os.listdir(roslogFolder):
      file_path = os.path.join(roslogFolder, the_file)
      if os.path.isfile(the_file):
          os.chmod(the_file, stat.S_IRWXU | stat.S_IRWXG | stat.S_IRWXO)
          os.remove(os.path.join(roslogFolder, the_file))
      elif os.path.islink(file_path):
          os.unlink(file_path)
      elif os.path.isdir(file_path):
          if len(os.listdir(file_path)) > 0:
              clean_dir(file_path)
          shutil.rmtree(os.path.join(roslogFolder, file_path))

os.system("killall -9 gazebo & killall -9 gzserver & killall -9 gzclient")
