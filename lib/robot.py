# TODO
# see why certainty is incorrect in histogram_grid, function
# get_certainty_at_discrete_point
# in the current robot.py, the histogram_grid_active_region does show the correct presence of the obstacle. There are some rounding errors!
# Anyway, there's a mismatch between the histogram_grid_active_region and what is returned by get_certainty_at_discrete_point to solve
# /*
# * NOTE: it is important to distinguish between the same variables at
# * t versus t-1. Instance variables are shared across two timesteps.
# */

# NOTE: all speed and velocity units are continuous distance per timestep.tgm
import matplotlib.patches as patches
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import math
import time
import numpy as np
from IPython import display
import warnings
import itertools


from lib.path_planner import PathPlanner
from lib.histogram_grid import HistogramGrid
from lib.polar_histogram import PolarHistogram

class Robot:
    def __init__(self, histogram_grid, polar_histogram, init_location, target_location, init_speed):
        # CHANGED: we shouldn't need polar_histogram, only histogram_grid
        self.path_planner = PathPlanner(histogram_grid, polar_histogram, init_location, target_location,mov_avg_l=5)
        self.target_location = target_location
        self.location = init_location
        self.discrete_location = init_location
        self.speed = init_speed
        self.update_angle()


    @classmethod
    def from_map(cls, map_fname, init_location, target_location, init_speed, active_region_dimension, resolution, num_bins):
        histogram_grid = HistogramGrid.from_map(map_fname, active_region_dimension, resolution, init_location)
        # print(f'robot: from_map: histogram_grid={histogram_grid}')
        polar_histogram = PolarHistogram(num_bins)
        return cls(histogram_grid, polar_histogram, init_location, target_location, init_speed)


    def update_angle(self):
        continuous_displacement = (self.target_location[0] - self.location[0], self.target_location[1] - self.location[1])
        continuous_robot_to_target_angle = math.atan2(continuous_displacement[1], continuous_displacement[0])
        self.angle = self.path_planner.get_best_angle(continuous_robot_to_target_angle)
        self.continuous_robot_to_target_angle = continuous_robot_to_target_angle


    def set_speed(self, speed):
        self.speed = speed


    def update_velocity(self):
        angle_radian = self.angle * math.pi/180
        # old_v_x, old_v_y = self.velocity
        self.velocity = (self.speed * math.cos(angle_radian), self.speed * math.sin(angle_radian))

    def update_location(self):
        angle_radian = self.angle * math.pi/180
        velocity_x, velocity_y = self.velocity

        old_x, old_y = self.location
        self.location = (old_x + velocity_x, old_y + velocity_y)
        # self.location = (old_x * math.cos(angle_radian)+ velocity_x, old_y * math.sin(angle_radian)+ velocity_y)

        # Why does path_planner need discrete location?
        self.discrete_location = self.path_planner.histogram_grid.continuous_point_to_discrete_point(self.location)
        print(f"robot: location,discrete_location = ",end="")
        print(f'{self.location,self.discrete_location}')
        self.path_planner.set_robot_location(self.discrete_location)


    # Main function per timestep
    # 1. Get angle from nothing at t=0, then
    # 2. get speed from nothing at t=0.
    # 3. Given position at 0, draw simulation at t=0,
    # 4. Now move from t=0 to t=1 by only updating the robot's position.
    def step(self, draw=True):
        self.print_polar_histogram()
        self.update_angle() # angle: Null (or optionally, t-1) => t
        # self.set_speed() # speed: Null (or optionally, t-1) => t
        print("\nrobot: step: best angle =", self.angle )
        self.update_velocity()
        self.update_location() # position: t => t+1

    def loop(self, num_steps, draw=True,anim_plot=True):
        print(f'\nrobot: in loop\n')
        plt.ion() # enable interactive plotting mode
        # add the polar histogram as a regular bar graph (maybe temporally for debug)
        if draw == True:
            figure, (simulation_plot,  polar_histogram_plot,polar_plot,histogram_grid_plot) = plt.subplots(1, 4, figsize=(18, 6))

            # 1. Plot the simulation
            obstacles_x, obstacles_y = self.path_planner.histogram_grid.get_obstacles() # get a list of points [(x1, y1), (x2, y2), ...]
            paths_robot = simulation_plot.scatter(*self.location, color='blue')
            paths_target = simulation_plot.scatter(*self.path_planner.target_location, color='green')
            paths_obstacles = simulation_plot.scatter(obstacles_x, obstacles_y, color='red')
            active_region_min_x, active_region_min_y, active_region_max_x, active_region_max_y = self.path_planner.histogram_grid.get_active_region(self.location)
            rectangle = simulation_plot.add_patch(
                patches.Rectangle(
                    (active_region_min_x, active_region_min_y),
                    active_region_max_x - active_region_min_x,
                    active_region_max_y - active_region_min_y,
                    fill=False
                )
            )
            simulation_plot.invert_yaxis()


            # 2. Plot the polar histogram
            num_bins = self.path_planner.polar_histogram.num_bins
            valley_threshold = self.path_planner.valley_threshold
            # polar_histogram_by_angle contains [(angle0, certainty0),(angle1,certainty1),...]
            polar_histogram_by_angle = self.path_planner.polar_histogram.get_angle_certainty()



            # NOTE: instead of sectors, get polar histogram bins and filter them by valley threshold
            bin_percentages = [1.0/num_bins for angle, certainty in polar_histogram_by_angle]
            bin_angles = [angle for angle, _ in polar_histogram_by_angle]
            bin_certainties = [certainty for angle, certainty in polar_histogram_by_angle]
            print(f'polar_histogram_by_angle nbel={len(polar_histogram_by_angle)},{polar_histogram_by_angle[0:2]}')
            polar_histogram_plot.bar(bin_angles,bin_certainties,width=10)
            

            colors = ['blue' if certainty < valley_threshold else 'red' for angle, certainty in polar_histogram_by_angle]
            labels = [f'{int(angle)}' for angle, certainty in polar_histogram_by_angle]
            # labels = [angle for angle, certainty in polar_histogram_by_angle]
            generator = enumerate(polar_histogram_by_angle)
            def make_autopct(bin_percentages):
                def my_autopct(pct):
                    index, (angle, certainty) = next(generator)
                    # return f'i{angle:.0f}:  {certainty:.1f}'
                    return f'{certainty:.0f}'
                    # return f'{int(angle)}:  {certainty:.1f}'
                    # return '{angle}:  {certainty:.1f}'.format(angle=int(angle), certainty=certainty)
                    # return '{angle:.0f}:  {certainty:.1f}'.format(angle=angle, certainty=certainty)
                return my_autopct

            pie_patches, pie_texts, pie_autotexts = polar_plot.pie(bin_percentages, colors=colors, labels=labels, startangle=0, counterclock=True, autopct=make_autopct(bin_percentages))


            # 3. Plot the valley
            histogram_grid_active_region = self.path_planner.histogram_grid.get_histogram_grid_active_region(active_region_min_x, active_region_min_y, active_region_max_x, active_region_max_y)
            print('active region histogram =')
            print(*histogram_grid_active_region, sep='\n')
            histogram_grid_plot.clear()
            histogram_grid_plot.matshow(histogram_grid_active_region, origin="upper")
            if anim_plot:
                display.clear_output(wait=True) # Uncomment for animation or Comment to see all steps.
            display.display(plt.gcf())


        # loop over the required steps
        for i in range(num_steps):
            print(f'STEP {i}/{num_steps}')
            self.step()
            if draw == True:

                # 1. Replot the simulation
                # TODO: no need to get the obstacles at every steps! Do it once and for all(?)
                obstacles_x, obstacles_y = self.path_planner.histogram_grid.get_obstacles()
                simulation_plot.clear()
                rectangle = simulation_plot.add_patch(
                    patches.Rectangle(
                        (active_region_min_x, active_region_min_y),
                        active_region_max_x - active_region_min_x,
                        active_region_max_y - active_region_min_y,
                        fill=False
                    )
                )
                paths_robot = simulation_plot.scatter(*self.location, color='blue')
                paths_target = simulation_plot.scatter(*self.path_planner.target_location, color='green')
                paths_obstacles = simulation_plot.scatter(obstacles_x, obstacles_y, color='red')
                active_region_min_x, active_region_min_y, active_region_max_x, active_region_max_y = self.path_planner.histogram_grid.get_active_region(self.location)
                rectangle.set_bounds(active_region_min_x, active_region_min_y, active_region_max_x - active_region_min_x, active_region_max_y - active_region_min_y)
                # Draw direction vector from current location to target
                # angle_to_target = math.atan2(self.target_location[1]-self.discrete_location[1],self.target_location[0]-self.discrete_location[0])
                # angle_to_target = self.path_planner.histogram_grid.get_angle_between_discrete_points( self.discrete_location, self.target_location)
                # print(f'INFO: angle_to_target = {angle_to_target}')
                angle_to_target = math.atan2(self.target_location[1]-self.discrete_location[1],self.target_location[0]-self.discrete_location[0]) *180/3.14
                print(f'INFO: angle_to_target = {angle_to_target}')
                # angle_to_target = self.angle
                # print(f'INFO: angle_to_target = {angle_to_target}')
                # angle_to_target = self.path_planner.histogram_grid.get_angle_between_discrete_points( self.target_location, self.discrete_location)
                # direction_vector = np.array([np.sin(angle_to_target), np.cos(angle_to_target)])
                direction_vector = np.array([np.cos(angle_to_target*3.14/180.0), np.sin(angle_to_target*3.14/180.0)])
                simulation_plot.quiver(self.discrete_location[0], self.discrete_location[1],  direction_vector[0], direction_vector[1], scale = 5,angles = 'xy', color='g')
                # Draw adjustment direction vector taking into account obstacle
                direction_vector = np.array([np.cos(self.angle*3.14/180.0), np.sin(self.angle*3.14/180.0)])
                simulation_plot.quiver(self.discrete_location[0], self.discrete_location[1],  direction_vector[0], direction_vector[1],scale = 10, angles = 'xy',color='r')
                locstr = f'{self.discrete_location},{self.angle:.0f}°'
                simulation_plot.text(self.discrete_location[0]+5, self.discrete_location[1],locstr) # str(i))
                locstr = f'{angle_to_target:.0f}°'
                simulation_plot.text(5, 5,locstr) # str(i))
                # simulation_plot.text(self.discrete_location[0], self.discrete_location[1],r'salut')
                simulation_plot.invert_yaxis()


                # 2. Replot the polar histogram
                # sectors = self.path_planner.get_sectors() # NOTE: sectors are only valid
                num_bins = self.path_planner.polar_histogram.num_bins
                valley_threshold = self.path_planner.valley_threshold
                polar_histogram_by_angle = self.path_planner.polar_histogram.get_angle_certainty()
                # NOTE: instead of sectors, get polar histogram bins and filter them by valley threshold
                bin_percentages = [1.0/num_bins]*len(polar_histogram_by_angle)
                # bin_percentages = [1.0/num_bins for angle, certainty in polar_histogram_by_angle]
                bin_angles = [angle for angle, _ in polar_histogram_by_angle]
                bin_certainties = [certainty for angle, certainty in polar_histogram_by_angle]
                # print(f'bin_percentages {bin_percentages}')
                polar_histogram_plot.clear()
                polar_histogram_plot.bar(bin_angles,bin_certainties,width=10)
                colors = ['blue' if certainty < valley_threshold else 'red' for angle, certainty in polar_histogram_by_angle]
                labels = [angle for angle, certainty in polar_histogram_by_angle]
                generator = enumerate(polar_histogram_by_angle)
                def make_autopct(bin_percentages):
                    def my_autopct(pct):
                        index, (angle, certainty) = next(generator)
                        return '{certainty:.0f}'.format(certainty=certainty)
                        # return '{certainty:.1f}'.format(certainty=certainty)
                    return my_autopct

                polar_plot.clear()
                # polar_plot.pie(bin_percentages, colors=colors, labels=labels, startangle=0, counterclock=True, autopct=make_autopct(bin_percentages))
                polar_plot.pie(bin_percentages, colors=colors, labels=labels, startangle=0, autopct=make_autopct(bin_percentages))


                # 3. Replot the histogram_grid
                histogram_grid_active_region = self.path_planner.histogram_grid.get_histogram_grid_active_region(active_region_min_x, active_region_min_y, active_region_max_x, active_region_max_y)
                print('active region histogram =')
                print(*histogram_grid_active_region, sep='\n')
                histogram_grid_plot.clear()
                histogram_grid_plot.matshow(histogram_grid_active_region, origin="upper")


                # 4. Actually display the plots
                print(f'plot at step {i}/{num_steps}')

                if anim_plot:
                    display.clear_output(wait=True) # NOTE: Uncomment this for animation. Comment this out if you want to see all steps.
                display.display(plt.gcf())


    def print_polar_histogram(self):
        self.path_planner.print_polar_histogram()


    def get_polar_bins(self):
        return self.polar_histogram.getPolarBins()


def concatenate(*lists):
    return itertools.chain(*lists)
