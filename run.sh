#!/bin/sh

gnome-terminal -e "./ProcessVideo"
echo "REMOTE CODE"
ssh jpanda "/home/jon/kinect/project/run.sh"
