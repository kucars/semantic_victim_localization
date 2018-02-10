#!/usr/bin/env python

import rospy
from victim_localization.msg import IterationInfo
from numpy import mean, array, hypot, diff, convolve, arange, sin, cos, ones, pi, matrix
import time
import signal
import sys
import csv
import os
import rospkg

from matplotlib import pyplot

f, ax_plot = pyplot.subplots(4,4)

# Define as dict so we can support values from multiple methods
file_prefix = time.strftime("%Y-%m-%d_%H-%M-%S_", time.localtime())
files_csv = {} #Array of open csv files
#dir_path = os.path.dirname(os.path.realpath(__file__)) # current directory path
rospack  = rospkg.RosPack()
dir_path = rospack.get_path('victim_localization')
#dir_path = os.path.dirname(os.path.realpath(__file__)) # current directory path

print (dir_path)

iterations = {}
distance = {}
distance_inc = {}
entropy_total = {}
entropy_total_dl = {}
entropy_total_thermal = {}
entropy_total_wireless = {}
time_iteration = {}
time_iteration_total= {}
curr_max_prob = {}
curr_max_loc_x = {}
curr_max_loc_y = {}
generator_type = {}
selected_utility = {}
selected_utility_dl = {}
selected_utility_thermal = {}
selected_utility_wireless = {}


def main():
  global file_prefix
  #if ( len(sys.argv) > 1):
    #file_prefix = sys.argv[1] + "_" + file_prefix

  rospy.init_node('plot_iteration_info', anonymous=True)
  rospy.Subscriber("victim_localization/iteration_info", IterationInfo, callback)

  try:
    while (True):
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
        ax_plot[3][0].clear()
        ax_plot[3][1].clear()
        ax_plot[3][2].clear()
        ax_plot[3][3].clear()


        for key in iterations:
          ax_plot[0][0].plot(iterations[key], entropy_total[key], label=key)
          ax_plot[0][1].plot(iterations[key], entropy_total_dl[key], label=key)
          ax_plot[0][2].plot(iterations[key], entropy_total_thermal[key], label=key)
          ax_plot[0][3].plot(iterations[key], entropy_total_wireless[key], label=key)
          ax_plot[1][0].plot(iterations[key], selected_utility[key], label=key)
          ax_plot[1][1].plot(iterations[key], selected_utility_dl[key], label=key)
          ax_plot[1][2].plot(iterations[key], selected_utility_thermal[key], label=key)
          ax_plot[1][3].plot(iterations[key], selected_utility_wireless[key], label=key)

          ax_plot[2][0].plot(iterations[key], generator_type[key], label=key)
          ax_plot[2][1].plot(iterations[key], time_iteration[key], label=key)
          ax_plot[2][2].plot(iterations[key], time_iteration_total[key], label=key)
          ax_plot[2][3].plot(iterations[key], distance[key], label=key)
          ax_plot[3][0].plot(iterations[key], curr_max_prob[key], label=key)
          ax_plot[3][1].plot(curr_max_loc_x[key], curr_max_loc_y[key], label=key)
          #ax_plot[3][2].plot(curr_max_loc_x[key], curr_max_loc_y[key], label=key)
          #ax_plot[3][3].plot(curr_max_loc_x[key], curr_max_loc_y[key], label=key)


        ax_plot[0][0].set_ylabel('Global Entropy Combined')
        ax_plot[0][1].set_ylabel('Global Entropy DL')
        ax_plot[0][2].set_ylabel('Global Entropy Thermal')
        ax_plot[0][3].set_ylabel('Global Entropy Wireless')

        ax_plot[1][0].set_ylabel('Utility(combine)')
        ax_plot[1][1].set_ylabel('Utility(DL)')
        ax_plot[1][2].set_ylabel('Utility(thermal)')
        ax_plot[1][3].set_ylabel('Utility(wireless)')

        ax_plot[2][0].set_ylabel('Generator Type')
        ax_plot[2][1].set_ylabel('Iteration Time (s)')
        ax_plot[2][2].set_ylabel('Total Time (s)')
        ax_plot[2][3].set_ylabel('Distance Travelled (m)')
        ax_plot[3][0].set_ylabel('Current Max Victim prob')
        ax_plot[3][1].set_ylabel('Current Max Victim loc Y (m)')

        ax_plot[0][0].set_xlabel('Iterations')
        ax_plot[0][1].set_xlabel('Iterations')
        ax_plot[0][2].set_xlabel('Iterations')
        ax_plot[0][3].set_xlabel('Iterations')
        ax_plot[1][0].set_xlabel('Iterations')
        ax_plot[1][1].set_xlabel('Iterations')
        ax_plot[1][2].set_xlabel('Iterations')
        ax_plot[1][3].set_xlabel('Iterations')
        ax_plot[2][0].set_xlabel('Iterations')
        ax_plot[2][1].set_xlabel('Iterations')
        ax_plot[2][2].set_xlabel('Iterations')
        ax_plot[2][3].set_xlabel('Iterations')
        ax_plot[3][0].set_xlabel('Iterations')
        ax_plot[3][1].set_xlabel('Current Max Victim loc X (m)')
        #ax_plot[3][2].set_xlabel('Iterations')
        #ax_plot[2][1].set_xlabel('Current Max Victim loc X (m)')


        # Add legends
        legends = []
        legends.append( ax_plot[0][0].legend(loc='upper center', bbox_to_anchor=(0.5, 1.3), shadow=True) )

        # Set the fontsize
        for leg in legends:
          if leg is not None:
            for label in leg.get_texts():
              label.set_fontsize('small')

        pyplot.pause(1)


  except KeyboardInterrupt:
    print ("Exitting plotting node")


def callback(data):
  method = data.method_selection + " ~ " + data.method_generation

  # Check if method is already defined as key in dict
  if not method in iterations:
    # Create blank arrays
    iterations[method] = []
    distance[method] = []
    distance_inc[method] = []
    entropy_total[method] = []
    entropy_total_dl[method] = []
    entropy_total_thermal[method] = []
    entropy_total_wireless[method] = []

    time_iteration[method] = []
    time_iteration_total[method] = []
    generator_type[method] = []
    curr_max_prob[method] = []
    curr_max_loc_x[method] = []
    curr_max_loc_y[method] = []

    selected_utility[method] = []
    selected_utility_dl[method] = []
    selected_utility_thermal[method] = []
    selected_utility_wireless[method] = []

    # Open csv file in append mode
    files_csv[method] = open(dir_path+"/Data/"+file_prefix + method + "combined" + ".csv", "a")
    csvwriter = csv.writer(files_csv[method], delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
    csvwriter.writerow([
      'Iteration',
      'Entropy Total',
      'Entropy Total DL',
      'Entropy Total thermal',
      'Entropy Total wireless',
      'Utility',
      'Utility DL',
      'Utility Thermal',
      'Utility Wireless',
      'Entropy Change %',
      'Distance Travelled',
      'Time Iteration (s)',
      'Total Time (s)',
      'Generator Type',
      'Current Max Victim Prob',
      'Current Max Victim loc X',
      'Current Max Victim loc Y',
      ])

  # Iterations went back in time. Indicates start of new NBV loop. Exit program
  if (len(iterations[method]) > 1 and
      data.iteration < iterations[method][-1]):
      exit_gracefully()

  iterations[method].append(data.iteration)
  entropy_total[method].append(data.entropy_total)
  entropy_total_dl[method].append(data.entropy_total_dl)
  entropy_total_thermal[method].append(data.entropy_total_thermal)
  entropy_total_wireless[method].append(data.entropy_total_wireless)

  distance[method].append(data.distance_total)
  time_iteration[method].append(data.time_iteration)
  curr_max_prob[method].append(data.curr_max_prob)
  curr_max_loc_x[method].append(data.curr_max_loc_x)
  curr_max_loc_y[method].append(data.curr_max_loc_y)

  generator_type[method].append(data.generator_type)

  selected_utility[method].append(data.selected_utility)
  selected_utility_dl[method].append(data.selected_utility_dl)
  selected_utility_thermal[method].append(data.selected_utility_thermal)
  selected_utility_wireless[method].append(data.selected_utility_wireless)

  entropy_change = '';
  if (len(entropy_total[method]) > 1):
    prev = entropy_total[method][-2]
    curr = entropy_total[method][-1]
    entropy_change = (curr - prev)/((curr + prev)/2) * 100

    distance_inc[method].append(distance[method][-1] - distance[method][-2])
  else:
    distance_inc[method].append(0)

  if (len(time_iteration[method]) > 1):
    time_iteration_total[method].append(time_iteration_total[method][-1]+time_iteration[method][-1])

  else:
    time_iteration_total[method].append(time_iteration[method][-1])

  csvwriter = csv.writer(files_csv[method], delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
  csvwriter.writerow([
    data.iteration,
    entropy_total[method][-1],
    entropy_total_dl[method][-1],
    entropy_total_thermal[method][-1],
    entropy_total_wireless[method][-1],
    data.selected_utility,
    data.selected_utility_dl,
    data.selected_utility_thermal,
    data.selected_utility_wireless,
    entropy_change,
    data.distance_total,
    data.time_iteration,
    time_iteration_total[method][-1],
    data.generator_type,
    data.curr_max_prob,
    data.curr_max_loc_x,
    data.curr_max_loc_y,
    ]
    )

def cleanup_before_exit():
  print("[Plot] Closing all csv files")
  # Close any open csv files
  for key, file in files_csv.items():
    file.close()

def exit_gracefully(signum = None, frame = None):
  cleanup_before_exit()
  os._exit(1)

if __name__ == '__main__':
  # Workaround to force plots to close when pressing CTRL-C
  original_sigint = signal.getsignal(signal.SIGINT)
  signal.signal(signal.SIGINT, exit_gracefully)

  try:
    main()
  except Exception, e:
    cleanup_before_exit()
    #print("Exception: " + sys.exc_info()[1])
    print("Exception: " + str(e))
print('Application terminated')
