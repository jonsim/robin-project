#!/bin/sh

# AUTHOR:      Jonathan Simmonds
# DATE:        12/12/12
# DESCRIPTION: THIS FILE SHOULD LIVE ON THE HOST (DESKTOP) MACHINE.
#              Syncs data from the jpanda's working directory to the desktop's git repo.

#echo "Syncing with jpanda (PULLING changes)."
#rsync -vlptgu jon@192.168.43.132:~/robot/* ~/individual_project
echo "Syncing with jpanda (PUSHING changes)."
rsync -vlptgu ~/individual_project/* jon@192.168.43.132:~/robot
