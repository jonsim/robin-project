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
from numpy import zeros, int16
import struct

#--------------------------------------------------------------#
#------------------------ MAIN FUNCTION -----------------------#
#--------------------------------------------------------------#

# Print a picture
# Variables
image_width  = 640
image_height = 480

# Create (draw) the image (bg = black)
img = Image.new("RGB", (image_width, image_height), (0,255,0))
draw = ImageDraw.Draw(img)

# Load the data into an array
"""
print "Loading data..."
x = 0
y = 0
v = zeros((640, 480), dtype=int16)
with open('fc_640x480_d.csv', 'r') as f:
    for line in f:
        line_split = line.split()
        for split in line_split:
            v[x][y] = int(split)
            x += 1
        x = 0
        y += 1
"""
print "Loading data..."
v = zeros((640, 480), dtype=int16)
x = 0
y = 0
with open("fc_640x480_d.dat", "rb") as f:
    byte = f.read(2)
    while byte:
        v[x][y] = struct.unpack('H', byte)[0]
        x += 1
        if x == 640:
            x = 0
            y += 1
        byte = f.read(2)


# Draw onto the image from the array
#scaling_factor = 255.0 / max_v
scaling_factor = 255.0 / 4000.0
print "Drawing image (scaling_factor = %.5f) ..." % (scaling_factor)
for y in range(image_height):
    for x in range(image_width):
        gs_colour = int(v[x][y] * scaling_factor)
        if gs_colour > 255:
            rgb_colour = (255, 0, 0)
        else:
            rgb_colour = (gs_colour, gs_colour, gs_colour)
        draw.point((x,y), fill=rgb_colour)


# Save the image
img.show()
img.save("image_test.png")
