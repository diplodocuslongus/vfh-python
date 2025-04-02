import matplotlib.pyplot as plt
import sys
sys.path.insert(1, '..')

# from lib.polar_histogram import PolarHistogram
# from lib.histogram_grid import HistogramGrid
# from lib.path_planner import PathPlanner
from lib.robot_standalone import Robot

# HistogramGrid params
# active_region_dimension = (16, 16)
active_region_dimension = (16, 8)
resolution = 1 # 1cm per node
# map_fname = 'map.txt'
map_fname = '../map_no_sides.txt'

# PolarHistogram params
num_bins = 36 # each bin is 360/num_bins degrees
target_location = (50, 50)

# Robot Params
init_location = (1, 1)
init_speed = 1

robot = Robot.from_map(map_fname, init_location, 
                       target_location, init_speed, active_region_dimension, resolution, num_bins)

num_steps = 5

robot.loop(num_steps, draw=True)
