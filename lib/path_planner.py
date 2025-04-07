"""
path_planner.py

PathPlanner should cannibalize both histogram_grid and polar_histogram. There
is no reason that
"""
import warnings
import math
from itertools import groupby
from operator import itemgetter

# PolarHistogram class creates an object to represent the Polar Histogram


class PathPlanner:
    def __init__(self, histogram_grid, polar_histogram, robot_location, target_location, th_certainty=200, b=1, mov_avg_l=5,
                 s_max=15, valley_threshold=200):
        """
        Creates a Polar Histogram object with the number of bins passed.

        Args:
            polar_histogram: Object used to store the polar histogram.
            histogram_grid: Object used to store the grid/map of obstacles.
            th_certainty, b, mov_avg_l: Hyperparameters for smoothing the polar histogram.
            s_max: Hyperparameter: the maximum number of nodes that define a wide valley
        """
        self.polar_histogram = polar_histogram
        self.histogram_grid = histogram_grid
        self.set_target_discrete_location(target_location)
        self.th_certainty = th_certainty
        self.b = b
        self.mov_avg_l = mov_avg_l
        self.s_max = s_max
        self.valley_threshold = valley_threshold
        self.target_location = target_location
        # self.robot_location = histogram_grid.get_robot_location()
        # self.target_discrete_location = histogram_grid.get_target_discrete_location()
        self.set_robot_location(robot_location)

#     //TODO: Add ability to dynamically set certainty value
#     //TODO This function may be deprecated as we restructure the robot code for ROSMOD
    def set_robot_location(self, robot_location):
        """new_location: a tuple (x, y)."""
        # self.histogram_grid.set_robot_location(robot_location)
        self.generate_histogram(robot_location)

    def set_target_discrete_location(self, target_discrete_location):
        self.histogram_grid.set_target_discrete_location(
            target_discrete_location)

    def generate_histogram(self, robot_location):
        robot_to_target_angle = self.histogram_grid.get_angle_between_discrete_points(
            self.target_location, robot_location)

        """Builds the vector field histogram based on current position of robot and surrounding obstacles"""
        self.polar_histogram.reset()

        print(f"path_planner:generate_histogram:robot_location{robot_location},sensor active region", end="")
        active_region_min_x, active_region_min_y, active_region_max_x, active_region_max_y = self.histogram_grid.get_active_region(
            robot_location)
        print(f"{self.histogram_grid.get_active_region(robot_location)}")
        histogram_grid = self.histogram_grid
        polar_histogram = self.polar_histogram
        print(f'\tcertainty, distance, delta_certainty from histogram_grid=')
        # scan the active region (model the range of the sensor)
        # so far the sensor is a LIDAR (surrounds the robot)
        for x in range(active_region_min_x, active_region_max_x):
            for y in range(active_region_min_y, active_region_max_y):
                node_considered = (x, y)
                certainty = histogram_grid.get_certainty_at_discrete_point(
                    node_considered)
                distance = histogram_grid.get_continuous_distance_between_discrete_points(
                    node_considered, robot_location)
                delta_certainty = (certainty ** 2) * \
                    (self.th_certainty - self.b * distance)
                # print(f'certainty, distance, delta_certainty=')
                print(f'\t{certainty}, {distance:.1f}, {delta_certainty:.1f}')
                robot_to_node_angle = histogram_grid.get_angle_between_discrete_points(
                    robot_location, node_considered)
                # shows the angle between robot's current location and the node (section of the obstacle)
                print(f"\trobot_to_node_angle: {robot_to_node_angle:0.1f}°",end="")
                print(f'\t({robot_location} to {node_considered})')
                # print(f'\t(robot_location {robot_location} to node {node_considered})')
                if delta_certainty != 0:
                    print(f"\tadding certainty {delta_certainty:.1f} to angle {robot_to_node_angle:0.1f}° (node{node_considered}")
                    polar_histogram.add_certainty_to_bin_at_angle(
                        robot_to_node_angle, delta_certainty)

        polar_histogram.smooth_histogram(self.mov_avg_l)

    # TODO: We need to reorganize the polar histogram, starting NOT with the
    # target angle but with first bin closest to the target angle which does
    # not have a certainty (due to distance) and ending at max length.

    def get_filtered_polar_histogram(self):
        '''
        returns:
            the indices for which the certainty is lower than the valley_threshold
        '''
        print("path_planner: unfiltered _polar_histogram =", *(f"{x:.1f}" for x in self.polar_histogram._polar_histogram))
        # print(f"path_planner: unfiltered _polar_histogram ={self.polar_histogram._polar_histogram}")
        filtered = [bin_index for bin_index, certainty in enumerate(
            self.polar_histogram._polar_histogram) if certainty < self.valley_threshold]
        print("path_planner: filtered < %s =" % self.valley_threshold, filtered)
        return filtered

    def get_sectors_from_filtered_polar_histogram(self, filtered_polar_histogram):
        #  extract sectors (groups of consecutive non-zero elements) from the circular histogram
        # TODO: each sector needs to be sorted by wrapped angle.
        # this may not be doing what is expected....
        return [list(map(itemgetter(1), g)) for k, g in groupby(enumerate(filtered_polar_histogram), lambda ix: ix[0] - ix[1])]

    def get_sectors(self):
        filtered_polar_histogram = self.get_filtered_polar_histogram()
        sectors = self.get_sectors_from_filtered_polar_histogram(
            filtered_polar_histogram)
        return sectors

    def get_obstacles(self):
        return self.histogram_grid.get_obstacles()

    def get_best_angle(self, robot_to_target_angle):
        sectors = self.get_sectors()
        print(f'path_planner: get_best_angle: nb sectors ={len(sectors)}')
        print(f'\tnb sectors ={len(sectors)} get_best_angle: sectors ={sectors}')
        if len(sectors) == 0:
            # raise ValueError('path_planner: the entire histogram is a valley, given valley threshold ' + str(self.valley_threshold))
            least_likely_bin = sorted(range(len(self.polar_histogram._polar_histogram)),
                                      key=lambda k: self.polar_histogram._polar_histogram[k])[0]
            middle_angle = self.polar_histogram.get_middle_angle_of_bin(
                least_likely_bin)
            warnings.warn("path_planner: the entire polar histogram is a valley (given threshold = %s), setting best angle to least likely bin middle angle = %s" % (
                self.valley_threshold, middle_angle))
            return middle_angle

        # Edge Case: there is only one sector and whole histogram < valley threshold. 
        # So simply follow angle = angle to target
        if len(sectors) == 1:
            # raise ValueError('path_planner: the entire histogram is a valley, given vallye threshold ' + str(self.valley_threshold))
            print(f"path_planner: one sector and the entire histogram is < valley_threshold {self.valley_threshold}, setting best angle to robot_to_target_angle = {robot_to_target_angle / math.pi * 180:0.2f}°")
            warnings.warn(f"path_planner: the entire histogram is below valley_threshold {self.valley_threshold}, setting best angle to robot_to_target_angle = {robot_to_target_angle / math.pi * 180:0.2f}°")
            return robot_to_target_angle / math.pi * 180

        angles = []
        for sector in sectors:
            if len(sector) > self.s_max:
                # Case 1: Wide valley. Include only s_max bins.
                # k_n is the bin closest to the target direction
                if abs(sector[0] - robot_to_target_angle) > abs(sector[-1] - robot_to_target_angle):
                    k_n = 0
                    k_f = k_n + self.s_max - 1
                else:
                    k_n = len(sector) - 1
                    k_f = k_n - self.s_max + 1

            else:
                # Case 2: Narrow valley. Include all bins.
                # Order doesn't matter.
                k_n = sector[0]
                k_f = sector[-1]

            # print("k_n =", k_n)
            # print("k_f =", k_f)
            angle = (self.polar_histogram.get_middle_angle_of_bin(
                k_n) + self.polar_histogram.get_middle_angle_of_bin(k_f)) / 2
            # print("path_planner: angle =", angle)
            angles.append(angle)

        # print("robot_to_target_angle =", robot_to_target_angle)
        distances = [(angle, abs(robot_to_target_angle - angle))
                     for angle in angles]
        # print("path_planner: distances =", distances)
        smallest_distances = sorted(distances, key=itemgetter(1))
        # print("path_planner: smallest_distances =", smallest_distances)
        return smallest_distances[0][0]

    def print_histogram(self):
        print('path_planner: print PolarHistogram:')
        print(self.polar_histogram)

    def get_object_grid(self):
        return self.histogram_grid.get_object_grid()

    def get_cell_value(self, i, j):
        return self.histogram_grid.get_cell_value(i, j)

    def get_i_max(self):
        return self.histogram_grid.get_i_max()

    def get_j_max(self):
        return self.histogram_grid.get_j_max()
