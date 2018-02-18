
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

#rospack  = rospkg.RosPack()
dir = '/home/abdulrahman/catkin_ws/src/victim_localization/Results'
methods = {}

#define Utility name in a dic
Utility = {"Entropy" : "Utility1= $Entropy$", "ExpDist":"Utility2= $Entropy*e^{(- \lambda*d)}$", "MAX" : "Utility3= $argmax(P)*Entropy*e^{(- \lambda*d)}$"
           ,"NNWW1" : "Utility4= $(U_{exp}+U_{victim})*e^{(- \lambda*d)}$" ,  "NNWW3" : "Utility5= $(U_{exp}+2*U_{victim})*e^{(- \lambda*d)}$" ,  "NNWW2" : "Utility6= $(U_{exp}+2*U_{victim})$"}

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
  method_name = []
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
      
      
  for m in methods:
    folder_method = os.path.join(dir,m)

    os.chdir( folder_method )
    for name in sorted( os.listdir(".") ):
      if os.path.isdir(name):
        folder_run = os.path.join(folder_method,name)
        run_stats = ExtractRunData(folder_run)
        methods[m].append( run_stats )
        
        if "Entropy" in name:
          run_stats.method_name=Utility["Entropy"]
        elif "ExpDist" in name:
          run_stats.method_name=Utility["ExpDist"]
        elif "MAX" in name:
          run_stats.method_name=Utility["MAX"]
        elif "NNW1" in name:
          run_stats.method_name=Utility["NNW1"]
        elif "NNWW1" in name:
          run_stats.method_name=Utility["NNWW1"]
        elif "NNWWNOEXP" in name:
          run_stats.method_name=Utility["NNWWNOEXP"]
        else:
          run_stats.method_name=name
          
        os.chdir( folder_method )
  for key, method in sorted(methods.items()):
    f, ax_plot = pyplot.subplots(2,3)
    ax_plot[0][0].clear()
    ax_plot[0][1].clear()
    ax_plot[0][2].clear()
    ax_plot[1][1].clear()
    ax_plot[1][0].clear()
    ax_plot[1][2].clear()
        
    Number_of_method=0;
    for r in method:      
      Number_of_method=Number_of_method+1
      print Number_of_method
    
    for r in range(Number_of_method):

      ax_plot[0][0].plot(method[r].iterations, method[r].entropy_total, label=method[r].method_name)
      ax_plot[0][1].plot(method[r].iterations, method[r].generator_type,label=method[r].method_name)
      ax_plot[0][2].plot(method[r].iterations, method[r].total_time,label=method[r].method_name)
      ax_plot[1][0].plot(method[r].iterations, method[r].distance,label=method[r].method_name)
      ax_plot[1][1].plot(method[r].iterations, method[r].max_prob,label=method[r].method_name)
    
    
      ax_plot[0][0].set_ylabel('Global Entropy')
      ax_plot[0][1].set_ylabel('Generator Type')
      ax_plot[0][2].set_ylabel('Total Time (s)')
      ax_plot[1][0].set_ylabel('Distance Travelled (m)')
      ax_plot[1][1].set_ylabel('Current Max Victim prob')
    
      ax_plot[0][0].set_xlabel('Iterations')
      ax_plot[0][1].set_xlabel('Iterations')
      ax_plot[0][2].set_xlabel('Iterations')
      ax_plot[1][0].set_xlabel('Iterations')
      ax_plot[1][0].set_xlabel('Current Max Victim loc X (m)')
    
    
    # Add legends
    f.suptitle(key,fontsize=16)
    legends = []
    legends.append( ax_plot[0][0].legend(loc='upper center', bbox_to_anchor=(0.5, 1.3), shadow=True) )
    pyplot.autoscale(enable=True, axis='y', tight='true')
    pyplot.savefig(dir + "/" + key+ 'figure.png', format='png', dpi=900)


              
                      


if __name__ == '__main__':
  main()
