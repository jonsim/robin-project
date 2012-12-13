#!/bin/sh

# AUTHOR:      Jonathan Simmonds
# DATE:        12/12/12
# DESCRIPTION: Syncs data from the jpanda's working directory to the desktop's git repo.

echo "Syncing with jpanda (pulling changes)."
rsync -vlptgu -e "ssh" jpanda:~/kinect/OpenNI/Platform/Linux/Redist/OpenNI-Bin-Dev-Linux-Arm-v1.5.4.0/Samples/NiSimpleRead/* .
