#!/usr/bin/python

from matplotlib.font_manager import FontProperties                      
from matplotlib.lines import Line2D  
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import sys

TEXT_SIZE  = 13
TITLE_SIZE = 18
AXIS_OFFSET = 1.0

def data_plotter (points1, points2, labels1, labels2, lines, title, output_filename=None):
    global TEXT_SIZE, TITLE_SIZE, AXIS_OFFSET
    
    x_values1 = []
    y_values1 = []
    x_values2 = []
    y_values2 = []
    for p in points1:
        x_values1.append(p[0])
        y_values1.append(p[1])
    for p in points2:
        x_values2.append(p[0])
        y_values2.append(p[1])
    
    # pre-configure the plot environment
    matplotlib.rcParams['axes.unicode_minus'] = False
    
    # create the plot
    fig = plt.figure()
    plt.subplots_adjust(left=0.1, right=0.9, top=0.9, bottom=0.1)
    ax = fig.add_subplot(111)
    fig.patch.set_facecolor('white')
    
    # plot the data
    # colors are green, blue, orange.
    colors = ['#bbe600', '#00c7d9', '#e68e00']
    textcolors = ['#777777', '#336699']
    #textcolors = ['k', 'k']
    textoffset = (0, 7.5)
    # lines
    for l in lines:
        ax.add_line(l)
    # points
    
    if len(points1) > 0:
        ax.scatter(x_values1, y_values1, s=20, c=colors[0], zorder=2, label="Nodes")
        # labels
        for i in range(0, len(labels1)):
            plt.annotate(labels1[i], xy=points1[i], xytext=textoffset, textcoords="offset points", ha="center", va="bottom", color=textcolors[0])
    #        plt.annotate(labels1[i], xy=points1[i], xytext=(-20,20), textcoords = 'offset points', ha = 'right', va = 'bottom', bbox = dict(boxstyle = 'round,pad=0.5', fc = 'yellow', alpha = 0.5), arrowprops = dict(arrowstyle = '->', connectionstyle = 'arc3,rad=0'))
    
    if len(points2) > 0:
        ax.scatter(x_values2, y_values2, s=80, c=colors[1], zorder=2, label="Tables")
        for i in range(0, len(labels2)):
            plt.annotate(labels2[i], xy=points2[i], xytext=textoffset, textcoords="offset points", ha="center", va="bottom", color=textcolors[1])
    
    #ax.legend(lines, legend_values, bbox_to_anchor=(0.0, 1.02, 1.0, 0.0), loc=3, ncol=3, mode="expand", borderaxespad=0.0, prop={'size':TEXT_SIZE})
    ax.legend(loc="upper left", prop={"size": TEXT_SIZE})
    
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
    """ax.set_xlim(x_values[0], x_values[-1])
    ax.set_ylim(0, 51)
    ax.set_xticks(range(x_values[0], x_values[-1]+1, 250))
    ax.set_yticks(range(0, 50+1, 10))"""
    ax.grid(True, c='#666666')
    ax.set_aspect('equal')
    #xlim = ax.get_xlim()
    xlim = (min(x_values1 + x_values2)-AXIS_OFFSET, max(x_values1 + x_values2)+AXIS_OFFSET)
    #ylim = ax.get_ylim()
    ylim = (min(y_values1 + y_values2)-AXIS_OFFSET, max(y_values1 + y_values2)+AXIS_OFFSET)
    xdist = xlim[1] - xlim[0]
    ydist = ylim[1] - ylim[0]
    axdist = max(xdist, ydist)
    newxlim = [xlim[0] - ((axdist - xdist) / 2), xlim[1] + ((axdist - xdist) / 2)]
    newylim = [ylim[0] - ((axdist - ydist) / 2), ylim[1] + ((axdist - ydist) / 2)]
    #newlim = [min(xlim[0], ylim[0]), max(xlim[1], ylim[1])]
    #if min(x_values1 + x_values2 + y_values1 + y_values2) >= 0:
    #    newlim[0] = 0
    #newlim[0] -= 100
    #newlim[1] += 100
    ax.set_xlim(newxlim)
    ax.set_ylim(newylim)
    
    # setup the labels
    # ax.set_title(title)
    #ax.text(0.5, 1.11, title, horizontalalignment='center', transform=ax.transAxes, size=TITLE_SIZE)
    ax.set_title(title, size=TITLE_SIZE)
    ax.set_xlabel('Distance (m)')
    ax.set_ylabel('Distance (m)')
    
    # return
    if output_filename == None:
        plt.show()
    else:
        plt.savefig(output_filename + ".pdf")
        plt.savefig(output_filename + ".svg")



#### MAIN CODE ####
# args
if len(sys.argv) > 1:
    filename = sys.argv[1]
else:
    filename = "map_output.txt"
filename = '.'.join(filename.split('.')[0:-1])
# load the datas
labels = []
nodes  = []
lines  = []
tables = []
# extract all the stuffs
current_node = 0
table_mode = False
with open(filename + ".txt") as f:
    for line in f:
        if table_mode:
            tables.append(int(line))
        else:
            if line[0] == 'T':
                table_mode = True
            elif line[0] != ' ':
                split_line = line.split(' ')
                current_node = int(split_line[0])
                labels.append(split_line[0])
                nodes.append( (int(split_line[1]) / 1000.0, int(split_line[2]) / 1000.0) )
            else:
                split_line = line[2:-1].split(' ')
                next_node = int(split_line[0])
                lines.append( (min(current_node, next_node), max(current_node, next_node)) )
# reduce the line count
i = 0
while i < len(lines):
    j = 0
    while j < len(lines):
        if i != j and lines[i] == lines[j]:
            lines.pop(j)
        j += 1
    i += 1
# convert to raw lines
edges = []
for l in lines:
    start = ( nodes[l[0]][0], nodes[l[1]][0] )
    end   = ( nodes[l[0]][1], nodes[l[1]][1] )
    edges.append(Line2D(start, end, zorder=1))
# reduce the tables thing
removed_items = 0
table_nodes = []
table_labels = []
for t in tables:
    index = t - removed_items
    table_nodes.append(nodes[index])
    table_labels.append(labels[index])
    nodes.pop(index)
    labels.pop(index)
    removed_items += 1
# plot it
print "Plotting " + str(len(nodes)) + "+" + str(len(table_nodes)) + " nodes, " + str(len(edges)) + " edges."
data_plotter(nodes, table_nodes, labels, table_labels, edges, "Plot of a map generated during exploration", filename)
