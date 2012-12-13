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
image_width   = 640
image_height  = 480
maximum_depth = 0



# Create (draw) the image (bg = black)
print "Initialising..."

# Depth
image_d = Image.new("RGB", (image_width, image_height), (0,255,0))
draw_d  = ImageDraw.Draw(image_d)

# Colour
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
        vd = struct.unpack('H', byte)[0]
        v_d[x][y] = vd
        if vd > maximum_depth:
            maximum_depth = vd
        x += 1
        if x == 640:
            x = 0
            y += 1
        byte = f.read(2)

# Colour data
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
depth_scaling = 255.0 / maximum_depth
for y in range(image_height):
    for x in range(image_width):
        gs_colour = int(v_d[x][y] * depth_scaling)
        if gs_colour > 255:
            rgb_colour = (255, 0, 0)    # Display out of gamut colours in red
        else:
            rgb_colour = (gs_colour, gs_colour, gs_colour)
        draw_d.point((x,y), fill=rgb_colour)
# Scale
scale_offset_x = 10
scale_offset_y = 10
scale_height   = 200
scale_width    = 65
gradient_scaling = 255.0 / (scale_height-30)
draw_d.rectangle([scale_offset_x, scale_offset_y, scale_offset_x+scale_width, scale_offset_y+scale_height], fill=(255,255,255), outline=(0,0,0))
for y in range(scale_height-30):
    for x in range(20):
        gradient_shade = int(y * gradient_scaling)
        draw_d.point((scale_offset_x+5+x, scale_offset_y+20+y), fill=(gradient_shade, gradient_shade, gradient_shade))
title_string = "DEPTH (mm)"
title_string_s = draw_d.textsize(title_string)
title_string_offset_x = scale_width / 2 - title_string_s[0] / 2
title_string_offset_x = 4   # Comment this out for a more accurate x offset (at the risk of slight-non-centering
title_string_offset_y = 2
draw_d.text((scale_offset_x+title_string_offset_x, scale_offset_y+title_string_offset_y), title_string, fill=(0,0,0))
draw_d.text((scale_offset_x+25, scale_offset_y+15), "- 0", fill=(0,0,0))
draw_d.text((scale_offset_x+25, scale_offset_y+scale_height-16), "- " + str(maximum_depth), fill=(0,0,0))

# Colour image
for y in range(image_height):
    for x in range(image_width):
        rgb_colour = v_c[x][y]
        draw_c.point((x,y), fill=rgb_colour)



# Save the images
print "Displaying images..."

# Depth
image_d.show()
image_d.save("fc_640x480_d.png")

# Colour
image_c.show()
image_c.save("fc_640x480_c.png")
