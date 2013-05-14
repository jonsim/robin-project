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
from matplotlib.font_manager import FontProperties
import matplotlib
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
def convert_rgb_to_string (rgb):
    r = int(rgb[0]*255)
    g = int(rgb[1]*255)
    b = int(rgb[2]*255)
    s = '#%02x%02x%02x' % (r, g, b)
    return s
# H' takes values between 0-1530
# H' =    0- 255  RGB=   255, 0-255, 0
# H' =  255- 510  RGB= 255-0,   255, 0
# H' =  510- 765  RGB=     0,   255, 0-255
# H' =  765-1020  RGB=     0, 255-0, 255
# H' = 1020-1275  RGB= 0-255,     0, 255
# H' = 1275-1530  RGB=   255,     0, 255-0
def convert_rgb_to_depth (rgb):
    r = rgb[0]
    g = rgb[1]
    b = rgb[2]
    v = 0
    
    if r == 0 and g == 0 and b == 0:
        return 0
    
    if r == 255:
        if b == 0:
            v = g
        else:
            v = 1275 + b
    elif g == 255:
        if r == 0:
            v =  510 + b
        else:
            v =  255 + r
    elif b == 255:
        if r == 0:
            v =  765 + g
        else:
            v = 1020 + r
    v = (v * (4800/1530))
    if v < 1:
        return 0
    else:
        return v + 400


def convert_depth_to_rgb (v):
    # check for zero
    if v <= 400:
        return (0, 0, 0)
    
    v -= 400
    v /= (4800/1530)
    if v >= 1530:
        return (1.0, 0.0, 0.0)
    """
    if v < 255:
        return (255, v, 0)
    elif v < 510:
        return (510-v, 255, 0)
    elif v < 765:
        return (0, 255, v-510)
    elif v < 1020:
        return (0, 1020-v, 255)
    elif v < 1275:
        return (v-1020, 0, 255)
    else:
        return (255, 0, 1530-v)"""
    if v < 255:
        return (1.0, v/255.0, 0.0)
    elif v < 510:
        return ((510-v)/255.0, 1.0, 0.0)
    elif v < 765:
        return (0.0, 1.0, (v-510)/255.0)
    elif v < 1020:
        return (0.0, (1020-v)/255.0, 1.0)
    elif v < 1275:
        return ((v-1020)/255.0, 0.0, 1.0)
    else:
        return (1.0, 0.0, (1530-v)/255.0)


def convert_depth_to_gs (v):
    gs = v / (10000/255)
    return (gs, gs, gs)


# loads a given image into a 2d list
def load_image (filename):
    # make the data structure and open the image
    data2d = zeros((640, 480))
    input_img   = Image.open(filename)
    
    # loop through the input image, saving the values
    for y in range(0, 480):
        for x in range(0, 640):
            data2d[x][y] = convert_rgb_to_depth(input_img.getpixel((x, y)))
    
    # return
    return data2d


def make_histogram (data2d, side='both'):
    # setup the ranges to accomodate the side we want to look at.
    # y 0-240 is top half of image, 241-480 is bottom half
    xstart = 0
    xend   = 640
    ystart = 0
    yend   = 480
    if side == 'left':
        xend = 320
    elif side == 'right':
        xstart = 320
    elif side == 'region':
        xstart = 64
        xend   = 576
        ystart = 80
        yend   = 280
    
    # make the data structure
    hist = zeros(10000)
    
    # loop through the input and build the histogram
    for y in range(ystart, yend):
        for x in range(xstart, xend):
            hist[data2d[x][y]] += 1
    
    # produce the histogram colour spectrum
    colours = []
    for i in range(0, 10000):
        colours.append(convert_depth_to_rgb(i))
    
    # print the histogram's stats
    """hist_range1 = 0
    for i in range(600, 900):
        hist_range1 += hist[i]
    hist_range2 = 0
    for i in range(900, 1200):
        hist_range2 += hist[i]
    print "hist[0] =", hist[0]
    print "hist[600-899]  =", hist_range1
    print "hist[900-1199] =", hist_range2"""
    
    # tidy up the histogram
    histy = hist.tolist()
    histx = range(0, 10000)
    histy[0] = 0
    
    # adjust the ranges
    for i in range(0, len(histx)):
        histx[i] /= 1000.0
    for i in range(0, len(histy)):
        histy[i] /= 1000.0
    
    # return
    return (histx, histy, colours)


def plot_histogram (plot_axes, hist):
    plot_axes.bar(hist[0], hist[1], width=1.0/1000.0, color=hist[2], edgecolor=hist[2])
#    ax.plot(histx, histy, 'b-')


def setup_plot ():
    # create the plot
    fig = plt.figure()
    plt.subplots_adjust(left=0.1, right=0.95, top=0.95, bottom=0.1)
    ax = fig.add_subplot(111)
    
    # remove the top and right borders
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.get_xaxis().tick_bottom()
    ax.get_yaxis().tick_left()

    # setup the font
    font = {'family': 'Quattrocento',
            'weight': 'bold',
            'size'  : 16}
    matplotlib.rc('font', **font)
    
    # return
    return ax


def setup_plot_labels (ax):
    # setup the labels
    ax.set_xlabel('Depth (m)')
    ax.set_ylabel('Frequency (000\'s of pixels)')
    #ax.set_title(r'$\mathrm{Histogram\ of\ IQ:}\ \mu=100,\ \sigma=15$')
    ax.set_xlim(0, 5)
    ax.set_ylim(0, 4)
    ax.set_yticks(range(0, 4+1))


# returns an array A = MxN points with M observations in N dimensions. A[i]=A[i,:] returns the ith
# observation, A[i,0] returns the ith observations's x value etc. A[:,0] returns all the x values.
def generate_3d_data (data2d):
    X_SUBSTEP = 4
    Y_SUBSTEP = 4
    Sp_over_F = (0.2084 / 120)
    r = zeros(((640/X_SUBSTEP)*(480/Y_SUBSTEP), 3))
    i = 0
    for y in range(0, 480, Y_SUBSTEP):
        for x in range(0, 640, X_SUBSTEP):
            z = data2d[x][y]
            if z != 0:
                r[i,0] = (x - 320) * z * Sp_over_F  # rX
                r[i,1] = (240 - y) * z * Sp_over_F  # rY
                r[i,2] = z                          # rZ
            i += 1
    return r

def generate_colors (data2d):
    X_SUBSTEP = 4
    Y_SUBSTEP = 4
    colors = []
    for y in range(0, 480, Y_SUBSTEP):
        for x in range(0, 640, X_SUBSTEP):
            v = data2d[x][y]
            colors.append(convert_rgb_to_string(convert_depth_to_rgb(v)))
    return colors

"""def plot_2d_data (data2d, colours=None, depth_scale=True):
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
        title_string_offset_x = 4   # Comment this out for a more accurate x offset (at the risk of slight-non-centering)
        title_string_offset_y = 2
        draw.text((scale_offset_x+title_string_offset_x, scale_offset_y+title_string_offset_y), title_string, fill=(0,0,0))
        draw.text((scale_offset_x+25, scale_offset_y+15), "- 0", fill=(0,0,0))
        draw.text((scale_offset_x+25, scale_offset_y+scale_height-16), "- " + str(max_depth), fill=(0,0,0))
    
    # show
    image.show()"""


def plot_3d_data (data3d, colours=None, filename=None):
    # pre-configure the plot environment
    matplotlib.rcParams['axes.unicode_minus'] = False
    
    fig = plt.figure()
    axe = Axes3D(fig)
    
    fig.patch.set_facecolor('white')
    font = {'family': 'Quattrocento',
            'weight': 'bold',
            'size'  : 16}
    matplotlib.rc('font', **font)
    #axe.grid(True, c='#000000')
    #axe.set_aspect('equal')
    axe.set_aspect(0.5)
    c2 = ['#bbe600', '#00c7d9', '#e68e00']
    
    if colours == None:
        axe.scatter(data3d[:,0], data3d[:,1], data3d[:,2], zdir='y', s=2000, c=(255,0,0),     lw=0, marker='o')
    else:
        axe.scatter(data3d[:,0], data3d[:,1], data3d[:,2], zdir='y', s=2000, c=colours, lw=0, marker='o')
    #axe.dist = 15
    axe.set_xlabel('X-axis')
    axe.set_xlim3d([-1200, 1200])
    axe.set_zlabel('Y-axis')
    axe.set_zlim3d([-200, 1000])
    axe.set_ylabel('Z-axis')
    axe.set_ylim3d([ 1000, 3400])
    
    # return
    if filename == None:
        plt.show()
    else:
        #plt.savefig(filename + ".pdf")
        plt.savefig(filename + ".svg")
        plt.savefig(filename + "_pc.png")




#--------------------------------------------------------------#
#------------------------ MAIN FUNCTION -----------------------#
#--------------------------------------------------------------#
# get command line args
filename = ""
side = 'both'
if len(sys.argv) < 2:
    print "ERROR: the program must be called as follows:\n  ./diagram_maker.py filename.png ['left' | 'right' | 'region' | 'both']"
    sys.exit()
elif len(sys.argv) > 2:
    side = sys.argv[2]
filename = sys.argv[1]

# setup

# do stuff
print "Loading image..."
image = load_image(filename)
"""print "Converting..."
data3d = generate_3d_data(image)
colors = generate_colors(image)
print "Plotting..."
plot_3d_data(data3d, colors)
"""
print "Making histogram..."
histogram = make_histogram(image, side=side)

print "Plotting histogram..."
ax = setup_plot()
plot_histogram(ax, histogram)
setup_plot_labels(ax)

print "Outputting histogram..."
# show stuff
#plt.show()
# save stuff
output_filename = filename.split('.')[0] + "_hist_" + side
#plt.savefig(output_filename + ".svg")
plt.savefig(output_filename + ".png")
