#!/bin/sh

# AUTHOR:      Jonathan Simmonds
# DATE:        12/12/12
# DESCRIPTION: THIS FILE SHOULD LIVE ON THE REMOTE (PANDABOARD).
#              First compiles (if necessary) before running and bringing any output files back.

make
cd ../Bin/Arm-Release/
./Sample-NiSimpleRead
cd -
mv ../Bin/Arm-Release/*.dat .
#mv ../Bin/Arm-Release/*.csv .
