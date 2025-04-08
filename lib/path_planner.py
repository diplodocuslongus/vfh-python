"""
path_planner.py

PathPlanner should cannibalize both histogram_grid and polar_histogram. There
is no reason that
"""
import warnings
import math
import numpy as np
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
            the thresholded polar_histogram with values of certainty lower than the valley_threshold            equal to zero
        '''
        print("path_planner: unfiltered _polar_histogram =", *(f"{x:.1f}" for x in self.polar_histogram._polar_histogram))
        filtered = np.array(self.polar_histogram._polar_histogram)
        filtered[filtered < self.valley_threshold] = 0.0
        print("path_planner: filtered < %s =" % self.valley_threshold, filtered)
        return filtered

    def get_filtered_polar_histogram_orig(self):
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

    def get_sectors_from_filtered_polar_histogram_(self, filtered_polar_histogram):
        #  extract sectors (groups of consecutive non-zero elements) from the circular histogram
        # TODO: each sector needs to be sorted by wrapped angle.
        # this may not be doing what is expected....
        return [list(map(itemgetter(1), g)) for k, g in groupby(enumerate(filtered_polar_histogram), lambda ix: ix[0] - ix[1])]

    def get_sectors_from_filtered_polar_histogram(self, filtered_polar_histogram):
        #  extract sectors (groups of consecutive non-zero elements) from the circular histogram
        return [list(map(itemgetter(1), g)) for k, g in groupby(enumerate(filtered_polar_histogram), lambda ix: ix[0] - ix[1])]

    # vectorized extract the sectors
    def extract_nonzero_sectors_np_(self,arr):
        is_non_zero = arr != 0
        # Pad with False at both ends to detect edges
        padded = np.pad(is_non_zero.astype(int), (1, 1), constant_values=0)
        diffs = np.diff(padded)

        # Sector starts where diff == 1, ends where diff == -1
        starts = np.where(diffs == 1)[0]
        ends = np.where(diffs == -1)[0]

        sectors = [arr[start:end].tolist() for start, end in zip(starts, ends)]
        return sectors

    def extract_nonzero_sectors_np(self,arr):
        is_non_zero = arr != 0
        padded = np.pad(is_non_zero.astype(int), (1, 1), constant_values=0)
        diffs = np.diff(padded)

        starts = np.where(diffs == 1)[0]
        ends = np.where(diffs == -1)[0]

        # Sector values and indices
        sectors = [arr[start:end].tolist() for start, end in zip(starts, ends)]
        sector_indices = [(start, end - 1) for start, end in zip(starts, ends)]  # inclusive end

        return sectors, sector_indices


    def get_sectors_(self):
        filtered_polar_histogram = self.get_filtered_polar_histogram()
        sectors = self.get_sectors_from_filtered_polar_histogram(
            filtered_polar_histogram)
        return sectors

    def get_sectors(self, desired_angle=0.0):
        filtered_polar_histogram = self.get_filtered_polar_histogram()
        print(f'filtered_polar_histogram = {filtered_polar_histogram}')
        n_bins = len(filtered_polar_histogram)
        bin_size = 360 / n_bins
        desired_index = round(desired_angle / bin_size) % n_bins

        # if all(v == 0 for v in filtered_polar_histogram):
        if np.all(filtered_polar_histogram == 0):
            return [0.0] * n_bins, [],[],0

        # rotated = filtered_polar_histogram[-desired_index:] + filtered_polar_histogram[:-desired_index]
        rotated = np.roll(filtered_polar_histogram, desired_index)
        # Get sectors (consecutive non-zero sequences)
        sectors,sectors_indx = self.extract_nonzero_sectors_np(rotated)
        sector_count = len(sectors)
        return rotated, sectors, sectors_indx, sector_count


    def get_obstacles(self):
        return self.histogram_grid.get_obstacles()

    def get_best_angle(self, robot_to_target_angle):
        print(f'self.get_sectors()={self.get_sectors()}' )
        rotttt,sectors,sector_indx,nb_sectors = self.get_sectors()
        print(f'path_planner: get_best_angle: ')
        print(f'\tnb sectors ={nb_sectors} sectors,sectors_indx ={sectors,sector_indx}')
        if nb_sectors == 0:
            # no sector or certainty lower than threshold, follow heading to target as if no obstacle
            return robot_to_target_angle/ math.pi * 180

        # Edge Case: there is only one sector and whole histogram < valley threshold. 
        # So simply follow angle = angle to target
        if nb_sectors == 1:
            print(f"path_planner: one sector")
            # get middle of the sector from the start and end index of the sector
            # mid_angle = sector_indx[1]*10 # for test, 10° for 36 bins
            # mid_angle = np.median(sector_indx)*10 # for test, 10° for 36 bins
            # move away from the sector (the obstacle)

            direction_angle = 0.5 * (sector_indx[0][1]*10 -180- robot_to_target_angle / math.pi * 180)
            return direction_angle

        angles = []
        for sector in sectors:
            if nb_sectors > self.s_max:
                # Case 1: Wide valley. Include only s_max bins.
                # k_n is the bin closest to the target direction
                if abs(sector[0] - robot_to_target_angle) > abs(sector[-1] - robot_to_target_angle):
                    k_n = 0
                    k_f = k_n + self.s_max - 1
                else:
                    k_n = nb_sectors - 1
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

    def get_best_angle_(self, robot_to_target_angle):
        sectors = self.get_sectors()
        print(f'path_planner: get_best_angle: ')
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
    def print_polar_histogram(self):
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
