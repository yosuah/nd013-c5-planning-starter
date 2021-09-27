## Project: Basic behavior and Motion planning using Carla
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Overview
---
Given an existing skeleton of a planner code, fill in the missing parts to implement two of the main components of a 
traditional hierarchical planner: The Behavior Planner and the Motion Planner. 

Both will work in unison to be able to:
* Avoid static objects (cars, bicycles and trucks) parked on the side of the road (but still invading the lane). The vehicle must avoid crashing with these vehicles by executing either a “nudge” maneuver.
* Handle any type of intersection (3-way,  4-way intersections and roundabouts) by STOPPING in all of them (by default)
* Track the centerline on the traveling lane.

Use the Carla simulator environment to verify the solution.

Solution
---
Finite state machine-based behavior planning for a minimal set of longitudinal cases:
follow lane, decelerate to stop and stopped (at intersection).

Motion planning using cubic spirals. Multiple possible spirals are generated at different offsets
from the lane center in order to ensure that at least some of them are feasible (both given the
constraints and static collision checking). 

Static object collision checking using multiple circles to cover the ego vehicle and obstacles.

Velocity generation for each spiral, taking into account stop positions
at intersections and acceleration limits. Velocity generation uses a combination of linear
velocity profiles (constant acceleration).

Selection of the best path using cost functions. The costs include collision checking, proximity to goal and 
proximity to lane center.

Unit tests were added to all parts of the code where I made changes.

In order to see all changes I made 
[compare it to the last commit of the framework](https://github.com/yosuah/nd013-c5-planning-starter/compare/976205277cbb3c6f5bda08f01522890eb4d6e3f3...HEAD).
All `NOTE` comments are by me (but `TODO` comments were already in the code).

![Sample screen capture from project](planning_screencap_lowres_short.gif)

(Video running at 4x speed of the simulation, though there is no technical reason why the simulation could not
be made faster.)

[Open higher quality video](planning_screencap_lowres_short.mp4)
