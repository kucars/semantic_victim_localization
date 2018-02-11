import rospy
from victim_localization.msg import IterationInfo
from numpy import mean, array, hypot, diff, convolve, arange, sin, cos, ones, pi, matrix, isfinite
import time
import signal
import sys
import csv
import os
import glob
import subprocess
import shutil

from matplotlib import pyplot

"""
Folder structure is assumed to be the following:
- root
-- Method1
--- Run1 (contains 1 csv)
--- Run2 (contains 1 csv)
--- Run3 (contains 1 csv)
-- Method2
--- Run1
--- Run2
...
"""

def cumsum(it):
    total = 0
    for x in it:
        total += x
        yield total


dir = '/home/abdulrahman/test_result'
methods = {}
#skip_coverage = False

#if (len(sys.argv) > 1):
  #skip_coverage = True

class RunStats(object):
  def __init__(self, name, folder, path):
    self.name = name
    self.path = path
    self.folder = folder

    self.iterations = []
    self.entropy_total = []
    self.entropy_total_vision = []
    self.entropy_total_thermal = []
    self.entropy_total_wireless = []
    self.utility = []
    self.iteration_time = []
    self.total_time = []
    self.generator_type = []
    self.max_prob = []


def ExtractRunData(folder):
  # Read first csv file
  os.chdir( folder )
  try:
    file = glob.glob("*.csv")[0]
  except:
    print("-----------")
    print("ERROR: No csv file in " + folder)
    print("-----------")
    raise

  file_no_ext = os.path.splitext(file)[0]
  file_path = os.path.join(folder,file)

  c_reader = csv.reader(open(file_path, 'r'), delimiter=',')
  next(c_reader, None)  # skip the headers

  # Initialize columns
  iterations = []
  entropy_total = []
  entropy_total_vision = []
  entropy_total_thermal = []
  entropy_total_wireless = []
  utility = []
  distance = []
  iteration_time = []
  total_time = []
  generator_type = []
  max_prob = []

  # Read off data
  for row in c_reader:
    iterations.append( int(row[0]) )
    entropy_total.append( float(row[1]) )
    entropy_total_vision.append( float(row[2]) )
    entropy_total_thermal.append( float(row[3]) )
    entropy_total_wireless.append( float(row[4]) )
    utility.append( float(row[5]) )
    distance.append( float(row[10]) )
    total_time.append( float(row[12]) )
    generator_type.append( float(row[13]) )
    max_prob.append( float(row[14]) )

    t = 0
    try:
      t = float(row[11])
      if not isfinite(t):
        t = 0
    except:
      # Failure happens if number is blank
      print('Failed to convert time to float in iteration ' + str(iterations[-1]) )

    iteration_time.append ( t )

  # Create final stats
  stats = RunStats(file_no_ext, folder, file_path)

  stats.iterations = iterations
  stats.entropy_total = entropy_total
  stats.entropy_total_vision = entropy_total_vision
  stats.entropy_total_thermal = entropy_total_thermal
  stats.entropy_total_wireless = entropy_total_wireless
  stats.utility = utility
  stats.iteration_time = iteration_time
  stats.distance= distance
  stats.total_time = total_time
  stats.generator_type = generator_type
  stats.max_prob = max_prob


  return stats

def getMethodData():
  global methods
  # Get method names
  os.chdir( dir )
  for name in os.listdir("."):
    if os.path.isdir(name):
      # Create method entry for each folder
      methods[name] = []

  # Get different runs for each method
  for m in methods:
    folder_method = os.path.join(dir,m)

    os.chdir( folder_method )
    for name in sorted( os.listdir(".") ):
      if os.path.isdir(name):
        folder_run = os.path.join(folder_method,name)
        run_stats = ExtractRunData(folder_run)
        methods[m].append( run_stats )

        os.chdir( folder_method ) # Return to method folder

def plotAttribute(total_runs, attr, label):
  # Display entropy in each run
  f, ax_plot = pyplot.subplots(total_runs,1,sharey=True, sharex=True)

  for r in range(total_runs):
    for key, method in sorted(methods.items()):
      if (r >= len(method)):
        continue

      data = eval("method[r]." + attr)

      if (total_runs > 1):
        ax_plot[r].plot(method[r].iterations, data, label=key)
        ax_plot[r].set_ylabel(label)
        ax_plot[r].legend(loc='lower left', bbox_to_anchor=(1.0, 0.0), shadow=True)
      else:
        ax_plot.plot(method[r].iterations, data, label=key)
        ax_plot.set_ylabel(label)
        ax_plot.legend(loc='lower left', bbox_to_anchor=(1.0, 0.0), shadow=True)

def main():
  global methods
  getMethodData()

  # Get number of runs
  total_runs = 0
  for m in methods:
    runs = len(methods[m])
    if (runs > total_runs):
      total_runs = runs

  # Display final statistcs
  print ("Method, Runs, Iterations, Density, Distance (m), Entropy Reduction, Total Time (s), Avg Time Per Iteration (ms), Coverage (res = 0.05m), Coverage (res = 0.10m), Coverage (res = 0.50m)")

  plotAttribute(total_runs, "entropy_total", "Total Entropy")
  plotAttribute(total_runs, "distance", "Distance (m)")
  plotAttribute(total_runs, "utility", "utility")


  pyplot.show()

if __name__ == '__main__':
  main()
