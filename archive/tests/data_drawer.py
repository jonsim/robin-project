#!/usr/bin/python

#--------------------------------------------------------------#
# DESCRIPTION                                                  #
#   Takes a csv (must be comma separated ONLY) where each line #
#   represents a single pixel row and each value represents    #
#   the z values for each corresponding pixel.                 #
# AUTHOR                                                       #
#     Jonathan Simmonds                                        #
#--------------------------------------------------------------#
from collections import namedtuple
from numpy import zeros
from scipy.cluster.vq import kmeans2
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import Image, ImageDraw
import sys
import struct



#--------------------------------------------------------------#
#--------------------- CLASS DEFINITIONS ----------------------#
#--------------------------------------------------------------#
Point = namedtuple('Point', ['x', 'y', 'z'])




#--------------------------------------------------------------#
#-------------------- FUNCTION DEFINITIONS --------------------#
#--------------------------------------------------------------#
def load_2d_data_ascii (filename, xres=640):
    res_step = 640 / xres
    r = []
    with open(filename) as f:
        full = f.read()
        lines = full.split('\n')
        for y in range(0, 480, res_step):
            split_line = lines[y].split(',')
            for x in range(0, 640, res_step):
                r.append(Point(x, y, int(split_line[x])))
    return r


def load_2d_data (filename):
    x = 0
    y = 0
    r = []
    with open(filename, "rb") as f:
        byte = f.read(2)
        while byte:
            z = struct.unpack('H', byte)[0]
            r.append(Point(x, y, z))
            x += 1
            if x == 640:
                x = 0
                y += 1
            byte = f.read(2)
    return r


def subsample_2d_data (data, sample_step=1):
    r = []
    for p in data:
        if (p.x % sample_step == 0) and (p.y % sample_step == 0):
            r.append(p)
    return r


# returns an array A = MxN points with M observations in N dimensions. A[i]=A[i,:] returns the ith
# observation, A[i,0] returns the ith observations's x value etc. A[:,0] returns all the x values.
def generate_3d_data (data2d):
    Sp_over_F = (0.2084 / 120)
    r = zeros((len(data2d), 3))
    for i in range(len(data2d)):
        z = data2d[i].z
        if z != 0:
            r[i,0] = (data2d[i].x - 320) * z * Sp_over_F  # rX
            r[i,1] = (240 - data2d[i].y) * z * Sp_over_F  # rY
            r[i,2] = z                                    # rZ
    return r


def cluster_3d_data (data3d):
    # kmeans
    k = 3
    k_iter = 10
    rCents, rIds = kmeans2(data3d, k, iter=k_iter)
    
    # generate colours
    colourList = ('#ff7f00',
                  '#7f007f',
                  '#7fff00',
                  '#ffff00',
                  '#ff0000',
                  '#ff00ff',
                  '#00ffff',
                  '#00ff00',
                  '#0000ff',
                  '#ff007f')
    rColours = ([colourList[i] for i in rIds])
    return rIds, rColours


def plot_2d_data (data2d, colours=None, depth_scale=True):
    # Initialise, calculating the resolution and creating the image (bg=green)
    res_step = data2d[1].x - data2d[0].x
    xres = 640 / res_step
    yres = 480 / res_step
    image = Image.new("RGB", (xres, yres), (0,255,0))
    draw  = ImageDraw.Draw(image)
    
    # calculate max depth
    max_depth = 0
    for data in data2d:
        if (data.z > max_depth):
            max_depth = data.z
    
    # Depth image
    depth_scaling = 255.0 / max_depth
    for i in range(len(data2d)):
        color = int(data2d[i].z * depth_scaling)
        if colours == None:
            draw.point((data2d[i].x/res_step, data2d[i].y/res_step), fill=(color, color, color))
        else:
            draw.point((data2d[i].x/res_step, data2d[i].y/res_step), fill=colours[i])
    
    # Scale
    if (depth_scale and xres == 640):
        scale_offset_x = 10
        scale_offset_y = 10
        scale_height   = 200
        scale_width    = 65
        gradient_scaling = 255.0 / (scale_height-30)
        draw.rectangle([scale_offset_x, scale_offset_y, scale_offset_x+scale_width, scale_offset_y+scale_height], fill=(255,255,255), outline=(0,0,0))
        for y in range(scale_height-30):
            for x in range(20):
                gradient_shade = int(y * gradient_scaling)
                draw.point((scale_offset_x+5+x, scale_offset_y+20+y), fill=(gradient_shade, gradient_shade, gradient_shade))
        title_string = "DEPTH (mm)"
        title_string_s = draw.textsize(title_string)
        title_string_offset_x = scale_width / 2 - title_string_s[0] / 2
        title_string_offset_x = 4   # Comment this out for a more accurate x offset (at the risk of slight-non-centering
        title_string_offset_y = 2
        draw.text((scale_offset_x+title_string_offset_x, scale_offset_y+title_string_offset_y), title_string, fill=(0,0,0))
        draw.text((scale_offset_x+25, scale_offset_y+15), "- 0", fill=(0,0,0))
        draw.text((scale_offset_x+25, scale_offset_y+scale_height-16), "- " + str(max_depth), fill=(0,0,0))
    
    # show
    image.show()


def plot_3d_data (data3d, colours=None):
    # calculate max/min values
    """max_x = -10000
    max_y = -10000
    max_z = -10000
    min_x =  10000
    min_y =  10000
    min_z =  10000
    for x_data in data3d[0]:
        if x_data < min_x:
            min_x = x_data
        elif x_data > max_x:
            max_x = x_data
    for y_data in data3d[1]:
        if y_data < min_y:
            min_y = y_data
        elif y_data > max_y:
            max_y = y_data
    for z_data in data3d[2]:
        if z_data == 0:
            continue
        elif z_data < min_z:
            min_z = z_data
        elif z_data > max_z:
            max_z = z_data"""
    
    if colours == None:
        axe.scatter(data3d[:,0], data3d[:,1], data3d[:,2], zdir='y', s=2000, c='b',     edgecolors='b',     marker='o')
    else:
        axe.scatter(data3d[:,0], data3d[:,1], data3d[:,2], zdir='y', s=2000, c=colours, edgecolors=colours, marker='o')
    #axe.dist = 15
    axe.set_xlabel('X-axis')
    axe.set_xlim3d([-1500, 1500])
    axe.set_zlabel('Y-axis')
    axe.set_zlim3d([-1500, 1500])
    axe.set_ylabel('Z-axis')
    axe.set_ylim3d([ 1000, 4000])
    plt.show()




#--------------------------------------------------------------#
#------------------------ MAIN FUNCTION -----------------------#
#--------------------------------------------------------------#
# globals
class_colours = 'a'

# Get filename from command line argument
if len(sys.argv) > 1:
    filename = sys.argv[1]
else:
    print "The program needs the name of the file to display."
    sys.exit()

# Initialise
print "Initialising..."
fig = plt.figure()
axe = Axes3D(fig)

# Load data
print "Loading 2D data (from " + filename + ")..."
data2d_hd = load_2d_data(filename)
data2d_ld = subsample_2d_data(data2d_hd, 8)
print "Projecting to 3D data..."
data3d = generate_3d_data(data2d_ld)

# Cluster data
classes, colours = cluster_3d_data(data3d)

# Display data
print "Plotting 2D data..."
plot_2d_data(data2d_hd)
plot_2d_data(data2d_ld, colours)
print "Plotting 3D data..."
plot_3d_data(data3d, colours)

# Exit
print "Done."
