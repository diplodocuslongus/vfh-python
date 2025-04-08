# Vector Field Histogram - Python Implementation

Vector Field Histogram is a robot obstacle avoidance and path planning algorithm. 

This is a fork of i[vfh-python](https://github.com/vanderbiltrobotics/vfh-python) orignally by Zhanwen (Phil) Chen.

Modifications include cleanup, various fixes, standalone animation, "better" plots (subjective!), comments... Also rewrote the sector determination and consequent path planning (i.e. direction of navigation).

The main purpose is to learn how the VFH algorithm operates for obstacle avoidance and (simple) path planning.

Some definitions and notions used in the VFH algorithm and in the code:
- sector: a group of consecutive bins (modulo 2pi) in the polar histogram in which the certainty of an obstacle is non zero


