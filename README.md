## Project: Basic behavior and Motion planning using Carla
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Overview
---
Given an existing skeleton of a planner code, fill in the missing parts to get basic behavior and motion
planning. Use the Carla simulator environment to verify the solution.

Solution
---
Finite state machine-based behavior planning for a minimal set of longitudinal cases:
follow lane, decelerate to stop and stopped (at intersection).

Motion planning using cubic spirals, generating multiple possible spirals at different offsets
from the lane center. Velocity generation for each spiral, taking into account stop positions
at intersections and acceleration limits. Selection of the best path using const functions.

Unit tests were added to all parts of the code where I made changes.

In order to see all changes I made 
[compare it to the last commit of the framework](https://github.com/yosuah/nd013-c5-planning-starter/compare/976205277cbb3c6f5bda08f01522890eb4d6e3f3...HEAD).
All `NOTE` comments are by me (but `TODO` comments were already in the code).

![Sample screen capture from project](screen_capture.gif)

[Open higher quality video](screen_capture.mp4)
