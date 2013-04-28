#!/usr/bin/python

from matplotlib.font_manager import FontProperties
import matplotlib
import matplotlib.pyplot as plt
import csv
import numpy as np

TEXT_SIZE  = 13
TITLE_SIZE = 18

def data_plotter (x_values, legend_values, datas, title, output_filename=None):
    global TEXT_SIZE, TITLE_SIZE
    
    # create the plot
    fig = plt.figure()
    plt.subplots_adjust(left=0.1, right=0.95, top=0.8, bottom=0.1)
    ax = fig.add_subplot(111)
    fig.patch.set_facecolor('white')
    
    # plot the data
    # colors are green, blue, orange.
    colors = ['#bbe600', '#00c7d9', '#e68e00']
    lines  = []
    for i in range(0,len(datas)):
        lines.append(plt.plot(x_values, datas[i], colors[i]))
        plt.setp(lines[i], linewidth=1.6)
    ax.legend(lines, legend_values, bbox_to_anchor=(0.0, 1.02, 1.0, 0.0), loc=3, ncol=3, mode="expand", borderaxespad=0.0, prop={'size':TEXT_SIZE})
    
    # remove the top and right borders
    """    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.get_xaxis().tick_bottom()
    ax.get_yaxis().tick_left()"""
    
    # setup the font
    font = {'family': 'Quattrocento',
#            'weight': 'bold',
            'size'  : TEXT_SIZE}
    matplotlib.rc('font', **font)
    
    # setup the axes
    ax.set_xlim(x_values[0], x_values[-1])
    ax.set_ylim(0, 51)
    ax.set_xticks(range(x_values[0], x_values[-1]+1, 250))
    ax.set_yticks(range(0, 50+1, 10))
    ax.grid(True)
    
    # setup the labels
    # ax.set_title(title)
    ax.text(0.5, 1.11, title, horizontalalignment='center', transform=ax.transAxes, size=TITLE_SIZE)
    ax.set_xlabel('Distance (mm)')
    ax.set_ylabel('Number of markers detected (out of 50)')
    
    # return
    if output_filename == None:
        plt.show()
    else:
        plt.savefig(output_filename + ".pdf")
        plt.savefig(output_filename + ".svg")



#### MAIN CODE ####
# load the datas
data = csv.reader(open('MarkerDetectionResults.csv'), delimiter=',')
legend_values = []
x_values = []
datas = []
row_counter = 0
for row in data:
    if row_counter == 1:
        x_values = map(int, row[7:-1])
    elif row_counter > 1:
        legend_values.append(row[0])
        datas.append(map(int, row[7:-1]))
    row_counter += 1

# plot some stuffs
print "Starting..."
data_plotter(x_values, legend_values[ 0: 3], datas[ 0: 3],
             "Number of markers detected at various distances\nwith differing classifier sizes",
             "marker_eval_classifier_sizes")
print "Done 1"
data_plotter(x_values, legend_values[ 3: 6], datas[ 3: 6],
             "Number of markers detected at various distances\nwith differing numbers of classifier split",
             "marker_eval_classifier_splits")
print "Done 2"
data_plotter(x_values, legend_values[ 6: 9], datas[ 6: 9],
             "Number of markers detected at various distances\nwith differing training set sizes",
             "marker_eval_training_sizes")
print "Done 3"
data_plotter(x_values, legend_values[ 9:12], datas[ 9:12],
             "Number of markers detected at various distances\nwith differing scaling factors",
             "marker_eval_scaling_factors")
print "Done 4"
data_plotter(x_values, legend_values[12:15], datas[12:15],
             "Number of markers detected at various distances\nwith differing minimum neighbour counts",
             "marker_eval_neighbour_counts")
print "Done 5"
data_plotter(x_values, legend_values[15:18], datas[15:18],
             "Number of markers detected at various distances\nwith different marker size",
             "marker_eval_marker_sizes")
print "Done 6"
data_plotter(x_values, legend_values[18:20], datas[18:20],
             "Number of markers detected at various distances\nwith differing marker style",
             "marker_eval_marker_styles")
print "Done 7"
