#!/bin/bash

RUN=false
# Get the options
while getopts ":r" option; do
   case $option in
      r) # run the code
         RUN=true;;
   esac
done

echo $RUN

pushd starter_files && cmake . && make
BUILD_OK=$?
popd
if [ "$BUILD_OK" -eq 0 ]; then
  ./starter_files/spiral_planner_test
  if [ "$RUN" = true ]; then
    ./starter_files/spiral_planner&
    sleep 1.0
    python3 simulatorAPI.py
   fi
fi
