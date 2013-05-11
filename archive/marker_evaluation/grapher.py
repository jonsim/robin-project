#!/usr/bin/python

from matplotlib.font_manager import FontProperties
import matplotlib
import matplotlib.pyplot as plt
import csv
import numpy as np
import math

TEXT_SIZE  = 13
TITLE_SIZE = 18

def data_plotter (x_values, legend_values, datas, big_range, big_legend, title, output_filename=None):
    global TEXT_SIZE, TITLE_SIZE
    
    # create the plot
    fig = plt.figure()
    if big_legend:
        plt.subplots_adjust(left=0.1, right=0.95, top=0.757, bottom=0.1)
    else:
        plt.subplots_adjust(left=0.1, right=0.95, top=0.800,  bottom=0.1)
    ax = fig.add_subplot(111)
    fig.patch.set_facecolor('white')
    
    # plot the data
    # colors are green, blue, orange.
    colors = ['#bbe600', '#00c7d9', '#e68e00', '#cc0000', '#8800cc', '#00cc00']
    lines  = []
    for i in range(0,len(datas)):
        lines.append(plt.plot(x_values, datas[i], colors[i]))
        plt.setp(lines[i], linewidth=1.6)
    if big_legend:
        ax.legend(lines, legend_values, bbox_to_anchor=(0.0, 1.02, 1.0, 0.0), loc=3, ncol=int(math.ceil(len(legend_values)/2.0)), mode="expand", borderaxespad=0.0, prop={'size':TEXT_SIZE})
    else:
        ax.legend(lines, legend_values, bbox_to_anchor=(0.0, 1.02, 1.0, 0.0), loc=3, ncol=len(legend_values), mode="expand", borderaxespad=0.0, prop={'size':TEXT_SIZE})
    
    # remove the top and right borders
    """    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.get_xaxis().tick_bottom()
    ax.get_yaxis().tick_left()"""
    
    # setup the font
    """font = {'family': 'Quattrocento',
#            'weight': 'bold',
            'size'  : TEXT_SIZE}
    matplotlib.rc('font', **font)"""
    
    # setup the axes
    ax.set_ylim(0, 51)
    ax.set_yticks(range(0, 50+1, 10))
    if big_range:
        ax.set_xlim(750, 6500)
        ax.set_xticks(range(1000, 6501, 500))
    else:
        ax.set_xlim(750, 2500)
        ax.set_xticks(range(750, 2501, 250))
    ax.grid(True)
    
    # setup the labels
    # ax.set_title(title)
    if big_legend:
        ax.text(0.5, 1.18, title, horizontalalignment='center', transform=ax.transAxes, size=TITLE_SIZE)
    else:
        ax.text(0.5, 1.11, title, horizontalalignment='center', transform=ax.transAxes, size=TITLE_SIZE)
    ax.set_xlabel('Distance (mm)')
    ax.set_ylabel('Number of markers detected (out of 50)')
    
    # return
    if output_filename == None:
        plt.show()
    else:
        plt.savefig(output_filename + ".pdf")
        #plt.savefig(output_filename + ".png")



#### MAIN CODE ####
# load the datas
data = csv.reader(open('MarkerDetectionResults2.csv'), delimiter=',')
legend_values = []
x_values = []
datas = []
row_counter = 0
for row in data:
    if row_counter == 1:
        x_values = map(int, row[7:31])
    elif row_counter > 1:
        legend_values.append(row[0])
        datas.append(map(int, row[7:31]))
    row_counter += 1

# plot some stuffs
print "Starting..."
data_plotter(x_values, legend_values[ 0: 3], datas[ 0: 3], False, False,
             "Number of markers detected at various distances\nwith differing classifier sizes",
             "marker_eval_classifier_sizes")
print "Done 1"
data_plotter(x_values, legend_values[ 3: 6], datas[ 3: 6], False, False,
             "Number of markers detected at various distances\nwith differing numbers of classifier split",
             "marker_eval_classifier_splits")
print "Done 2"
data_plotter(x_values, legend_values[ 6: 9], datas[ 6: 9], False, False,
             "Number of markers detected at various distances\nwith differing training set sizes",
             "marker_eval_training_sizes")
print "Done 3"
data_plotter(x_values, legend_values[ 9:12], datas[ 9:12], False, False,
             "Number of markers detected at various distances\nwith differing scaling factors",
             "marker_eval_scaling_factors")
print "Done 4"
data_plotter(x_values, legend_values[12:15], datas[12:15], False, False,
             "Number of markers detected at various distances\nwith differing minimum neighbour counts",
             "marker_eval_neighbour_counts")
print "Done 5"
data_plotter(x_values, legend_values[15:20], datas[15:20], True, True,
             "Number of markers detected at various distances\nwith different marker size",
             "marker_eval_marker_sizes")
print "Done 6"
data_plotter(x_values, legend_values[20:22], datas[20:22], False, False,
             "Number of markers detected at various distances\nwith differing marker style",
             "marker_eval_marker_styles")
print "Done 7"
data_plotter(x_values, legend_values[22:28], datas[22:28], False, True,
             "Number of markers detected at various distances\nwith differing Y axis rotations",
             "marker_eval_marker_angles")
print "Done 8"
data_plotter(x_values, legend_values[28:31], datas[28:31], False, False,
             "Number of markers detected at various distances\nusing differing classification feature sets",
             "marker_eval_classifier_mode")
print "Done 9"
