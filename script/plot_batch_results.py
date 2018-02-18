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
import rospkg
from pylab import rcParams
rcParams['figure.figsize'] = 13, 10

from matplotlib import pyplot

rospack  = rospkg.RosPack()
dir = '/home/abdulrahman/catkin_ws/src/victim_localization/NNFinalResut'
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

def main():
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
    run_stats = ExtractRunData(folder_method)
    methods[m].append( run_stats )
    
    
    # plot the combine map
    if run_stats.entropy_total_vision[0]!=0:
        f, ax_plot = pyplot.subplots(3,4)
        
        ax_plot[0][0].clear()
        ax_plot[0][1].clear()
        ax_plot[0][2].clear()
        ax_plot[0][3].clear()
        ax_plot[1][0].clear()
        ax_plot[1][1].clear()
        ax_plot[1][2].clear()
        ax_plot[1][3].clear()
        ax_plot[2][0].clear()
        ax_plot[2][1].clear()
        ax_plot[2][2].clear()
        ax_plot[2][3].clear()
        
        
        ax_plot[0][0].plot(run_stats.iterations, run_stats.entropy_total)
        ax_plot[0][1].plot(run_stats.iterations, run_stats.entropy_total_vision)
        ax_plot[0][2].plot(run_stats.iterations,  run_stats.entropy_total_thermal)
        ax_plot[0][3].plot(run_stats.iterations, run_stats.entropy_total_wireless)
        ax_plot[1][0].plot(run_stats.iterations, run_stats.utility)
        ax_plot[1][1].plot(run_stats.iterations, run_stats.generator_type)
        ax_plot[1][2].plot(run_stats.iterations, run_stats.iteration_time)
        ax_plot[1][3].plot(run_stats.iterations, run_stats.total_time)
        ax_plot[2][0].plot(run_stats.iterations, run_stats.distance)
        ax_plot[2][1].plot(run_stats.iterations, run_stats.max_prob)
    
    
        ax_plot[0][0].set_ylabel('Global Entropy Combined')
        ax_plot[0][1].set_ylabel('Global Entropy DL')
        ax_plot[0][2].set_ylabel('Global Entropy Thermal')
        ax_plot[0][3].set_ylabel('Global Entropy Wireless')
        ax_plot[1][0].set_ylabel('Utility')
        ax_plot[1][1].set_ylabel('Generator Type')
        ax_plot[1][2].set_ylabel('Iteration Time (s)')
        ax_plot[1][3].set_ylabel('Total Time (s)')
        ax_plot[2][0].set_ylabel('Distance Travelled (m)')
        ax_plot[2][1].set_ylabel('Current Max Victim prob')
    
        ax_plot[0][0].set_xlabel('Iterations')
        ax_plot[0][1].set_xlabel('Iterations')
        ax_plot[0][2].set_xlabel('Iterations')
        ax_plot[0][3].set_xlabel('Iterations')
        ax_plot[1][0].set_xlabel('Iterations')
        ax_plot[1][1].set_xlabel('Iterations')
        ax_plot[1][2].set_xlabel('Iterations')
        ax_plot[1][3].set_xlabel('Iterations')
        ax_plot[2][0].set_xlabel('Iterations')
        ax_plot[2][1].set_xlabel('Current Max Victim loc X (m)')
        
        ax_plot[0][0].autoscale(enable=True, axis='y', tight='true')
    
        # Add legends
        legends = []
        legends.append( ax_plot[0][0].legend(loc='upper center', bbox_to_anchor=(0.5, 1.3), shadow=True) )
    
        # Set the fontsize
        for leg in legends:
          if leg is not None:
            for label in leg.get_texts():
              label.set_fontsize('small')
              
                      
        pyplot.autoscale(enable=True, axis='y', tight='true')
        pyplot.savefig(folder_method + '/figure.png', format='png', dpi=1200)
        
    else:
        f, ax_plot = pyplot.subplots(3,3)
        
        ax_plot[0][0].clear()
        ax_plot[0][1].clear()
        ax_plot[0][2].clear()
        ax_plot[1][1].clear()
        ax_plot[1][0].clear()
        ax_plot[1][2].clear()
        ax_plot[2][0].clear()

        
        
        ax_plot[0][0].plot(run_stats.iterations, run_stats.entropy_total)
        ax_plot[0][1].plot(run_stats.iterations, run_stats.utility)
        ax_plot[0][2].plot(run_stats.iterations, run_stats.generator_type)
        ax_plot[1][0].plot(run_stats.iterations, run_stats.iteration_time)
        ax_plot[1][1].plot(run_stats.iterations, run_stats.total_time)
        ax_plot[1][2].plot(run_stats.iterations, run_stats.distance)
        ax_plot[2][0].plot(run_stats.iterations, run_stats.max_prob)
    
    
        ax_plot[0][0].set_ylabel('Global Entropy Combined')
        ax_plot[0][1].set_ylabel('Utility')
        ax_plot[0][2].set_ylabel('Generator Type')
        ax_plot[1][0].set_ylabel('Iteration Time (s)')
        ax_plot[1][1].set_ylabel('Total Time (s)')
        ax_plot[1][2].set_ylabel('Distance Travelled (m)')
        ax_plot[2][0].set_ylabel('Current Max Victim prob')
    
        ax_plot[0][0].set_xlabel('Iterations')
        ax_plot[0][1].set_xlabel('Iterations')
        ax_plot[0][2].set_xlabel('Iterations')
        ax_plot[1][0].set_xlabel('Iterations')
        ax_plot[1][1].set_xlabel('Iterations')
        ax_plot[1][2].set_xlabel('Iterations')
        ax_plot[2][0].set_xlabel('Current Max Victim loc X (m)')
        
    
        # Add legends
        legends = []
        legends.append( ax_plot[0][0].legend(loc='upper center', bbox_to_anchor=(0.5, 1.3), shadow=True) )
    
        # Set the fontsize
        for leg in legends:
          if leg is not None:
            for label in leg.get_texts():
              label.set_fontsize('small')
              
                      
        pyplot.autoscale(enable=True, axis='y', tight='true')
        pyplot.savefig(folder_method + '/figure.png', format='png', dpi=1200)

if __name__ == '__main__':
  main()
