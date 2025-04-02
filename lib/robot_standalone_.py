# to use with the test/test.py, standalone for animation
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
import warnings
import itertools
import matplotlib.animation as animation


from lib.path_planner import PathPlanner
from lib.histogram_grid import HistogramGrid
from lib.polar_histogram import PolarHistogram

class Robot:
    def __init__(self, histogram_grid, polar_histogram, init_location, target_location, init_speed):
        self.path_planner = PathPlanner(histogram_grid, polar_histogram, init_location, target_location)
        self.target_location = target_location
        self.location = init_location
        self.speed = init_speed
        self.update_angle()


    @classmethod
    def from_map(cls, map_fname, init_location, target_location, init_speed, active_region_dimension, resolution, num_bins):
        histogram_grid = HistogramGrid.from_map(map_fname, active_region_dimension, resolution, init_location)
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
        self.velocity = (self.speed * math.cos(angle_radian), self.speed * math.sin(angle_radian))

    def update_location(self):
        angle_radian = self.angle * math.pi/180
        velocity_x, velocity_y = self.velocity

        old_x, old_y = self.location
        self.location = (old_x + velocity_x, old_y + velocity_y)

        # path_planner needs discrete location
        discrete_location = self.path_planner.histogram_grid.continuous_point_to_discrete_point(self.location)
        print("robot: discrete_location =", discrete_location)
        self.path_planner.set_robot_location(discrete_location)


    # Main function per timestep
    # 1. Get angle from nothing at t=0, then
    # 2. get speed from nothing at t=0.
    # 3. Given position at 0, draw simulation at t=0,
    # 4. Now move from t=0 to t=1 by only updating the robot's position.
    def step(self, draw=True):
        self.print_histogram()
        self.update_angle() # angle: Null (or optionally, t-1) => t
        # self.set_speed() # speed: Null (or optionally, t-1) => t
        print("robot: step: best angle =", self.angle )
        self.update_velocity()
        self.update_location() # position: t => t+1

    def loop(self, num_steps, draw=True):
        plt.ion() # enable interactive plotting mode
        if draw == True:
            figure, (simulation_plot, polar_plot, histogram_grid_plot) = plt.subplots(1, 3, figsize=(18, 6))
            # time.sleep(1)

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
            polar_histogram_by_angle = self.path_planner.polar_histogram.get_angle_certainty()
            # NOTE: instead of sectors, get polar histogram bins and filter them by valley threshold
            bin_percentages = [1.0/num_bins for angle, certainty in polar_histogram_by_angle]
            bin_certainties = [certainty for angle, certainty in polar_histogram_by_angle]
            colors = ['blue' if certainty < valley_threshold else 'red' for angle, certainty in polar_histogram_by_angle]
            labels = [angle for angle, certainty in polar_histogram_by_angle]
            generator = enumerate(polar_histogram_by_angle)
            def make_autopct(bin_percentages):
                def my_autopct(pct):
                    index, (angle, certainty) = next(generator)
                    return '{angle:.0f}:  {certainty:.1f}'.format(angle=angle, certainty=certainty)
                return my_autopct

            pie_patches, pie_texts, pie_autotexts = polar_plot.pie(bin_percentages, colors=colors, labels=labels, startangle=0, counterclock=True, autopct=make_autopct(bin_percentages))


            # 3. Plot the valley
            histogram_grid_active_region = self.path_planner.histogram_grid.get_histogram_grid_active_region(active_region_min_x, active_region_min_y, active_region_max_x, active_region_max_y)
            # print('active region histogram =')
            # print(*histogram_grid_active_region, sep='\n')
            histogram_grid_plot.clear()
            histogram_grid_plot.matshow(histogram_grid_active_region, origin="upper")
            print('first plot')
            # plt.show(block=True)
            plt.show(block=False)
            # plt.show()
            time.sleep(1)


        for i in range(num_steps):

            self.step()
            if draw == True:
                # plt.gcf()
                # plt.gca()
                # figure.clf()
                # time.sleep(1)
                # plt.cla()
                # plt.clf()

                # 1. Replot the simulation
                obstacles_x, obstacles_y = self.path_planner.histogram_grid.get_obstacles()
                paths_robot = simulation_plot.scatter(*self.location, color='blue')
                paths_target = simulation_plot.scatter(*self.path_planner.target_location, color='green')
                paths_obstacles = simulation_plot.scatter(obstacles_x, obstacles_y, color='red')
                active_region_min_x, active_region_min_y, active_region_max_x, active_region_max_y = self.path_planner.histogram_grid.get_active_region(self.location)
                rectangle.set_bounds(active_region_min_x, active_region_min_y, active_region_max_x - active_region_min_x, active_region_max_y - active_region_min_y)


                # 2. Replot the polar histogram
                # sectors = self.path_planner.get_sectors() # NOTE: sectors are only valid
                num_bins = self.path_planner.polar_histogram.num_bins
                valley_threshold = self.path_planner.valley_threshold
                polar_histogram_by_angle = self.path_planner.polar_histogram.get_angle_certainty()
                # NOTE: instead of sectors, get polar histogram bins and filter them by valley threshold
                bin_percentages = [1.0/num_bins for angle, certainty in polar_histogram_by_angle]
                bin_certainties = [certainty for angle, certainty in polar_histogram_by_angle]
                colors = ['blue' if certainty < valley_threshold else 'red' for angle, certainty in polar_histogram_by_angle]
                labels = [angle for angle, certainty in polar_histogram_by_angle]
                generator = enumerate(polar_histogram_by_angle)
                def make_autopct(bin_percentages):
                    def my_autopct(pct):
                        index, (angle, certainty) = next(generator)
                        return '{certainty:.1f}'.format(certainty=certainty)
                    return my_autopct

                polar_plot.clear()
                # polar_plot.pie(bin_percentages, colors=colors, labels=labels, startangle=0, counterclock=True, autopct=make_autopct(bin_percentages))
                polar_plot.pie(bin_percentages, colors=colors, labels=labels, startangle=0, autopct=make_autopct(bin_percentages))


                # 3. Replot the histogram_grid
                histogram_grid_active_region = self.path_planner.histogram_grid.get_histogram_grid_active_region(active_region_min_x, active_region_min_y, active_region_max_x, active_region_max_y)
                # print('active region histogram =')
                # print(*histogram_grid_active_region, sep='\n')
                histogram_grid_plot.clear()
                histogram_grid_plot.matshow(histogram_grid_active_region, origin="upper")


                # 4. Actually display the plots
                # plt.gcf()
                # plt.pause(0.1)
                # plt.show()
                # figure.show() 
                # figure.canvas.draw()
                # plt.show(block=True)




                time.sleep(0.21)
                # display.display(plt.gcf())


    def print_histogram(self):
        self.path_planner.print_histogram()


    def get_polar_bins(self):
        return self.polar_histogram.getPolarBins()


def concatenate(*lists):
    return itertools.chain(*lists)
