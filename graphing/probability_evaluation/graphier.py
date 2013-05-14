#!/usr/bin/python

from matplotlib.font_manager import FontProperties
import matplotlib
import matplotlib.pyplot as plt
import csv
import numpy as np

TEXT_SIZE  = 13
TITLE_SIZE = 18

def data_plotter (xs, y1s, y2s, output_filename=None):
    global TEXT_SIZE, TITLE_SIZE
    
    # rerange
    y1s = [y*100.0 for y in y1s]
    y2s = [y*100.0 for y in y2s]
    
    # create the plot
    fig = plt.figure()
    plt.subplots_adjust(left=0.1, right=0.95, top=0.8, bottom=0.1)
    ax = fig.add_subplot(111)
    fig.patch.set_facecolor('white')
    
    # plot the data
    # colors are green, blue, orange.
    legend_values = ["Pr(Greedy Node)", "Cumulative Pr(Greedy Node)"]
    colors = ['#bbe600', '#00c7d9', '#e68e00']
    lines  = []
    lines.append(plt.plot(xs, y1s, colors[0]))
    lines.append(plt.plot(xs, y2s, colors[1]))
    for line in lines:
        plt.setp(lines, linewidth=1.6)
    ax.legend(lines, legend_values, bbox_to_anchor=(0.0, 1.02, 1.0, 0.0), loc=3, ncol=2, mode="expand", borderaxespad=0.0, prop={'size':TEXT_SIZE})
    
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
    ax.set_xlim(0, 300)
    ax.set_ylim(0, 100)
    ax.set_xticks(range(0, 300+1, 60))
    ax.set_yticks(range(0, 100+1, 20))
    ax.grid(True)
    
    # setup the labels
    # ax.set_title(title)
    ax.text(0.5, 1.11, "Graph showing how probability of a Greedy Node action changes over time", horizontalalignment='center', transform=ax.transAxes, size=TITLE_SIZE)
    ax.set_xlabel('Time (seconds)')
    ax.set_ylabel('Probability (%)')
    
    # return
    if output_filename == None:
        plt.show()
    else:
        plt.savefig(output_filename + ".pdf")   # < proper text
        #plt.savefig(output_filename + ".svg")  # < path text



#### MAIN CODE ####
# load the datas
data = csv.reader(open("probabilities.csv"), delimiter=',')
x_values = []
y1_values = []
y2_values = []
row_counter = 0
for row in data:
    if row_counter > 0:
        x_values.append(int(row[0]))
        y1_values.append(float(row[2]))
        y2_values.append(float(row[-1]))
    row_counter += 1

# plot some stuffs
print "Starting..."
data_plotter(x_values, y1_values, y2_values, "graph")
print "Done"
