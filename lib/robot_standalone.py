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
    def __init__(self, histogram_grid, polar_histogram, init_location, target_location, init_velocity):
        self.path_planner = PathPlanner(histogram_grid, polar_histogram, init_location, target_location,mov_avg_l=3)
        self.target_location = target_location
        self.location = init_location
        self.velocity = (init_velocity,init_velocity)
        # self.velocity = init_velocity
        self.init_velocity = init_velocity
        self.update_angle()


    @classmethod
    def from_map(cls, map_fname, init_location, target_location, init_velocity, active_region_dimension, resolution, num_bins):
        histogram_grid = HistogramGrid.from_map(map_fname, active_region_dimension, resolution, init_location)
        polar_histogram = PolarHistogram(num_bins)
        return cls(histogram_grid, polar_histogram, init_location, target_location, init_velocity)


    def update_angle(self):
        continuous_displacement = (self.target_location[0] - self.location[0], self.target_location[1] - self.location[1])
        continuous_robot_to_target_angle = math.atan2(continuous_displacement[1], continuous_displacement[0])
        self.angle = self.path_planner.get_best_angle(continuous_robot_to_target_angle)
        self.continuous_robot_to_target_angle = continuous_robot_to_target_angle


    # def set_velocity(self, velocity):
    #     self.velocity = velocity


    def update_velocity(self):
        angle_radian = self.angle * math.pi/180
        self.velocity = (self.init_velocity * math.cos(angle_radian),self.init_velocity* math.sin(angle_radian))
        # self.velocity = (self.velocity[0] * math.cos(angle_radian), self.velocity[1] * math.sin(angle_radian))
        # self.velocity = (self.velocity * math.cos(angle_radian), self.velocity * math.sin(angle_radian))

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
    # 2. get velocity from nothing at t=0.
    # 3. Given position at 0, draw simulation at t=0,
    # 4. Now move from t=0 to t=1 by only updating the robot's position.
    def step(self, draw=True):
        self.print_histogram()
        self.update_angle() # angle: Null (or optionally, t-1) => t
        # self.set_velocity() # velocity: Null (or optionally, t-1) => t
        print("robot: step: best angle =", self.angle )
        self.update_velocity()
        self.update_location() # position: t => t+1

    def loop(self, num_steps, draw=True, anim_plot=True):
        if draw:
            fig, (simulation_plot,  polar_histogram_plot,polar_plot,histogram_grid_plot) = plt.subplots(1, 4, figsize=(18, 6))
            # fig, (simulation_plot, polar_plot, histogram_grid_plot) = plt.subplots(1, 3, figsize=(18, 6))

            # Initialize simulation plot
            obstacles_x, obstacles_y = self.path_planner.histogram_grid.get_obstacles() # get a list of points [(x1, y1), (x2, y2), ...]
            paths_robot, = simulation_plot.plot(*self.location, 'bo')
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
            # Initialize quiver plot for direction vector
            quiver_plot = simulation_plot.quiver(*self.location, 0, 0, color='red',angles='xy', scale=10)

            # Initialize path plot
            path_x, path_y = [self.location[0]], [self.location[1]]
            path_line, = simulation_plot.plot(path_x, path_y, color='blue', linestyle='-', linewidth=1)


            # Initialize polar histogram plot
            num_bins = self.path_planner.polar_histogram.num_bins
            valley_threshold = self.path_planner.valley_threshold
            # polar_histogram_by_angle contains: [(angle0, certainty0),(angle1,certainty1),...]
            polar_histogram_by_angle = self.path_planner.polar_histogram.get_angle_certainty()
            bin_percentages = [1.0 / num_bins for angle, certainty in polar_histogram_by_angle]
            bin_angles = [angle for angle, _ in polar_histogram_by_angle]
            bin_certainties = [certainty for angle, certainty in polar_histogram_by_angle]
            polar_histogram_plot.bar(bin_angles,bin_certainties,width=10)
            colors = ['blue' if certainty < valley_threshold else 'red' for angle, certainty in polar_histogram_by_angle]
            labels = [angle for angle, certainty in polar_histogram_by_angle]
            # generator = enumerate(polar_histogram_by_angle)
            # trick here: we create a new generator (local_generator) for each call. 
            # Ensures that the generator is reset for every frame of the animation
            # otherwise we have a 'StopIteration' error
            def make_autopct(bin_percentages):
                local_generator = enumerate(polar_histogram_by_angle) # Recreate the generator
                def my_autopct(pct):
                    index, (angle, certainty) = next(local_generator)
                    return '{certainty:.0f}'.format(certainty=certainty)
                return my_autopct


            pie_patches, pie_texts, pie_autotexts = polar_plot.pie(bin_percentages, colors=colors, labels=labels, startangle=0, autopct=make_autopct(bin_percentages))

            # Initialize histogram grid plot
            histogram_grid_active_region = self.path_planner.histogram_grid.get_histogram_grid_active_region(active_region_min_x, active_region_min_y, active_region_max_x, active_region_max_y)
            histogram_grid_plot.matshow(histogram_grid_active_region, origin="upper")

            def animate(i):
                self.step()

                # Update simulation plot
                obstacles_x, obstacles_y = self.path_planner.histogram_grid.get_obstacles()
                x_coords = [point[0] for point in [self.location]]
                y_coords = [point[1] for point in [self.location]]

                paths_robot.set_data(x_coords, y_coords)
                # paths_robot.set_data(self.location[0], self.location[1])
                active_region_min_x, active_region_min_y, active_region_max_x, active_region_max_y = self.path_planner.histogram_grid.get_active_region(self.location)
                rectangle.set_bounds(active_region_min_x, active_region_min_y, active_region_max_x - active_region_min_x, active_region_max_y - active_region_min_y)
                # Update quiver plot
                direction_x = np.cos(np.radians(self.angle))
                direction_y = np.sin(np.radians(self.angle))
                quiver_plot.set_UVC(direction_x, direction_y)
                quiver_plot.set_offsets(self.location)

                # Update path plot
                path_x.append(self.location[0])
                path_y.append(self.location[1])
                path_line.set_data(path_x, path_y)


                # Update polar histogram plot
                num_bins = self.path_planner.polar_histogram.num_bins
                valley_threshold = self.path_planner.valley_threshold
                polar_histogram_by_angle = self.path_planner.polar_histogram.get_angle_certainty()
                bin_percentages = [1.0 / num_bins for angle, certainty in polar_histogram_by_angle]
                bin_angles = [angle for angle, _ in polar_histogram_by_angle]
                bin_certainties = [certainty for angle, certainty in polar_histogram_by_angle]

                polar_histogram_plot.clear()
                polar_histogram_plot.bar(bin_angles,bin_certainties,width=10)
                colors = ['blue' if certainty < valley_threshold else 'red' for angle, certainty in polar_histogram_by_angle]
                labels = [f'{int(angle)}' for angle, certainty in polar_histogram_by_angle]
                generator = enumerate(polar_histogram_by_angle)
                polar_plot.clear()
                polar_plot.pie(bin_percentages, colors=colors, labels=labels, startangle=0, autopct=make_autopct(bin_percentages))

                # Update histogram grid plot
                histogram_grid_active_region = self.path_planner.histogram_grid.get_histogram_grid_active_region(active_region_min_x, active_region_min_y, active_region_max_x, active_region_max_y)
                histogram_grid_plot.clear()
                histogram_grid_plot.matshow(histogram_grid_active_region, origin="upper")

                return paths_robot, rectangle, pie_patches, histogram_grid_plot

            ani = animation.FuncAnimation(fig, animate, frames=num_steps, interval=300, blit=False) # Adjust interval as needed
            plt.show()

        else:
            for i in range(num_steps):
                self.step()

    def print_histogram(self):
        self.path_planner.print_histogram()


    def get_polar_bins(self):
        return self.polar_histogram.getPolarBins()


def concatenate(*lists):
    return itertools.chain(*lists)
