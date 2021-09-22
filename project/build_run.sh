#!/bin/bash

pushd starter_files && cmake . && make && ./spiral_planner_test; popd
./starter_files/spiral_planner&
sleep 1.0
python3 simulatorAPI.py
