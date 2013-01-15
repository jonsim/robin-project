#!/bin/sh

# AUTHOR:      Jonathan Simmonds
# DATE:        12/12/12
# DESCRIPTION: THIS FILE SHOULD LIVE ON THE HOST (DESKTOP) MACHINE.
#              Syncs data from the jpanda's working directory to the desktop's git repo.

echo "Syncing with jpanda (pulling changes)."
#rsync -vlptgu -e "ssh" jpanda:~/kinect/OpenNI/Platform/Linux/Redist/OpenNI-Bin-Dev-Linux-Arm-v1.5.4.0/Samples/NiSimpleRead/* .
rsync -vrlptgu -e "ssh" jpanda:~/kinect/project/* pandaboard/
cp pandaboard/*.dat .
