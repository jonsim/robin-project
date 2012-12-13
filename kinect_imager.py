#!/usr/bin/python

#--------------------------------------------------------------#
# DESCRIPTION                                                  #
#     Takes a csv with lines in the format 'x y v' representing#
#     the data from a 640x480 gs image. v is then scaled into  #
#     the drawable range to get a full spectrum from black to  #
#     white, and an image of the results printed               #
# AUTHOR                                                       #
#     Jonathan Simmonds                                        #
#--------------------------------------------------------------#
import sys
import Image, ImageDraw
from numpy import zeros, int16, int8
import struct

#--------------------------------------------------------------#
#------------------------ MAIN FUNCTION -----------------------#
#--------------------------------------------------------------#

# Print a picture
# Variables
image_width  = 640
image_height = 480



# Create (draw) the image (bg = black)
print "Initialising..."

# Depth
image_d = Image.new("RGB", (image_width, image_height), (0,255,0))
draw_d  = ImageDraw.Draw(image_d)

# Grayscale
image_c = Image.new("RGB", (image_width, image_height), (0,255,0))
draw_c  = ImageDraw.Draw(image_c)



# Load the data into an array
print "Loading data..."

# Depth data
v_d = zeros((image_width, image_height), dtype=int16)
x = 0
y = 0
with open("fc_" + str(image_width) + "x" + str(image_height) + "_d.dat", "rb") as f:
    byte = f.read(2)
    while byte:
        v_d[x][y] = struct.unpack('H', byte)[0]
        x += 1
        if x == 640:
            x = 0
            y += 1
        byte = f.read(2)

# Grayscale data
v_c = [ [0 for i in range(image_height)] for j in range(image_width) ]
x = 0
y = 0
with open("fc_" + str(image_width) + "x" + str(image_height) + "_c.dat", "rb") as f:
    br = f.read(1)
    bg = f.read(1)
    bb = f.read(1)
    while br and bg and bb:
        vr = struct.unpack('B', br)[0]
        vg = struct.unpack('B', bg)[0]
        vb = struct.unpack('B', bb)[0]
        v_c[x][y] = (vr, vg, vb)
        x += 1
        if x == 640:
            x = 0
            y += 1
        br = f.read(1)
        bg = f.read(1)
        bb = f.read(1)



# Draw onto the image from the array
print "Drawing images..."

# Depth image
depth_scaling = 255.0 / 4000.0
for y in range(image_height):
    for x in range(image_width):
        gs_colour = int(v_d[x][y] * depth_scaling)
        if gs_colour > 255:
            rgb_colour = (255, 0, 0)    # Display out of gamut colours in red
        else:
            rgb_colour = (gs_colour, gs_colour, gs_colour)
        draw_d.point((x,y), fill=rgb_colour)

# Grayscale image
#grayscale_scaling = float(2**8) / 2**16
for y in range(image_height):
    for x in range(image_width):
#        gs_colour = int(v_g[x][y] * grayscale_scaling)
#        if gs_colour > 255:
#            rgb_colour = (255, 0, 0)    # Display out of gamut colours in red
#        else:
#            rgb_colour = (gs_colour, gs_colour, gs_colour)
        rgb_colour = v_c[x][y]
        draw_c.point((x,y), fill=rgb_colour)



# Save the images
print "Displaying images..."

# Depth
image_d.show()
image_d.save("fc_640x480_d.png")

# Grayscale
image_c.show()
image_d.save("fc_640x480_c.png")
