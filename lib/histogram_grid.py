import numpy as np
import csv
from operator import sub # for get_distance_between_discrete_points
import math

class HistogramGrid:
    def __init__(self, dimension, resolution, robot_location, active_region_dimension):
        """
        dimension: Number of cells.
        resolution: Size of the cell in centimeters.
        """
        self.dimension = dimension
        self.resolution = resolution
        ncols, nrows = dimension
        self.histogram_grid = [[0] * ncols for r in range(nrows)]
        # self.object_grid = [[0] * ncols for i in range(nrows)]
        # CHANGED: Make Robot class the sole source of truth for location
        # self.robot_location = robot_location
        self.active_region_dimension = active_region_dimension

    @classmethod
    def from_map(cls, map_fname, active_region_dimension, resolution, robot_location):
        """
        Args:
            map_fname: text file containing the description of the map of obstacles
            active_region_dimension: a tuple (x, y).
        """
        with open(map_fname, 'r') as f:
            reader = csv.reader(f, delimiter=" ")
            lines = list(reader)

        # Convert string "1"s and "0"s to integer 1s and 0s
        lines = list(map(lambda l: list(map(int, l)), lines))
        # print("histogram_grid: histogram =")
        # print(*lines, sep="\n")
        dimension = (len(lines[0]), len(lines))
        hg = cls(dimension, resolution, robot_location, active_region_dimension)
        # print(f'histogram_grid: from_map: histogram_grid={hg.histogram_grid}')
        hg.histogram_grid = lines
        return hg


    def continuous_point_to_discrete_point(self, continuous_point):
        """
        Calculates in which node an object exists based on the continuous (exact) coordinates
        Returns a discrete point
        Args:
            continuous_point: A tuple ()
        """
        continuous_x, continuous_y = continuous_point
        discrete_x = continuous_x//self.resolution
        discrete_y = continuous_y//self.resolution
        # discrete_x = continuous_x//self.dimension[0]
        # discrete_y = continuous_y//self.dimension[1]
        # if(out.x < iMax && out.y < jMax) return out;
        # TODO ERROR HANDLING
        # throw;
        return discrete_x, discrete_y


    def update_certainty_at_continuous_point(self, continuous_point, certainty):
        """
        Updates the certainty value for the node at which the object is located.

        Args:
            continuous_point: the continuous point at which object is located.
            certainty: certainty value to set.
        """
        discrete_x, discrete_y = self.continuous_point_to_discrete_point(continuous_point)
        # self.histogram_grid[discrete_x][discrete_y] = certainty
        self.histogram_grid[discrete_y][discrete_x] = certainty


    def get_certainty_at_discrete_point(self, discrete_point):
        """
        Returns the certainty of an object being present at the given node
        """
        discrete_x, discrete_y = discrete_point
        # return self.histogram_grid[discrete_x][discrete_y]
        return self.histogram_grid[discrete_y][discrete_x]


    def get_continuous_distance_between_discrete_points(self, discrete_start, discrete_end):
        """
        Returns scalar distance between two discretePoints (pos1 & pos2) on the histogram grid
        """
        discrete_displacement = get_discrete_displacement(discrete_start, discrete_end)
        continuous_displacement = tuple(self.resolution * axis for axis in discrete_displacement)
        continuous_distance = math.sqrt(sum(axis**2 for axis in continuous_displacement))
        return continuous_distance

    @classmethod
    def get_angle_between_discrete_points_(cls, discrete_start, discrete_end):
        """
        Returns the angle between the 2 points in the regular XY referential system (OX is horizontal pointing right)
        """
        # discrete_displacement = get_discrete_displacement(discrete_start, discrete_end)
        ang = np.arctan2(discrete_end[1]-discrete_start[1],discrete_end[0]-discrete_start[0])
        return np.rad2deg(ang  % (2 * np.pi))
        # return np.rad2deg(ang) # % (2 * np.pi))
    def get_angle_between_discrete_points(cls, discrete_start, discrete_end):
        """
        Returns the angle between the line between pos2 and posRef and the horizontal along positive i direction.
        """
        # discrete_displacement = get_discrete_displacement(discrete_start, discrete_end)

        ang1 = np.arctan2(*discrete_start[::-1])
        ang2 = np.arctan2(*discrete_end[::-1])
        return np.rad2deg((ang1 - ang2) % (2 * np.pi))

    def get_active_region(self, robot_location):
        # REVIEW: the four coordinates are discrete?
        # robot_location_x, robot_location_y = self.robot_location
        # CHANGED: Make Robot class the sole source of truth for location
        robot_location_x, robot_location_y = robot_location
        # print("histogram_grid: self.robot_location =", self.robot_location)
        # print("histogram_grid: get_active_region: self.active_region_dimension =", self.active_region_dimension)
        active_region_x_size, active_region_y_size = self.active_region_dimension
        x_max, y_max = self.dimension

        active_region_min_x = robot_location_x - active_region_x_size / 2
        if active_region_min_x < 0:
            active_region_min_x = 0

        active_region_min_y = robot_location_y - active_region_y_size / 2
        if active_region_min_y < 0:
            active_region_min_y = 0

        active_region_max_x = robot_location_x + active_region_x_size / 2
        # if active_region_max_x >= active_region_x_size:
            # active_region_max_x = active_region_x_size
        if active_region_max_x >= x_max:
            # print("\nactive_region_max_x >= x_max\n")
            active_region_max_x = x_max

        active_region_max_y = robot_location_y + active_region_y_size / 2

        # if active_region_max_y >= active_region_y_size:
        #     active_region_max_y = active_region_y_size
        if active_region_max_y >= y_max:
            # print("\nactive_region_max_y >= y_max\n")
            active_region_max_y = y_max


        active_region = (int(round(active_region_min_x)), int(round(active_region_min_y)), int(round(active_region_max_x)), int(round(active_region_max_y)))

        # print("histogram_grid: active_region =", active_region)
        return active_region

    # CHANGED: Make Robot class the sole source of truth for location
    # def get_robot_location(self):
    #     return self.robot_location

    def get_target_discrete_location(self):
        return self.target_discrete_location

    # CHANGED: Make Robot class the sole source of truth for location
    # def set_robot_location(self, robot_location):
    #     self.robot_location = robot_location

    def set_target_discrete_location(self, target_discrete_location):
        self.target_discrete_location = target_discrete_location

    # Get points(tuples)
    def get_obstacles(self):

        obstacles_points_x = []
        obstacles_points_y = []
        for row_idx, row in enumerate(self.histogram_grid):
            for col_idx, cell in enumerate(row):
                if cell != 0:
                    # print("histogram_grid: obstacle =", (row_idx, col_idx))
                    obstacles_points_x.append(col_idx)
                    obstacles_points_y.append(row_idx)
        # print("histogram_grid: obstacles_points =", list(zip(obstacles_points_x, obstacles_points_y)))
        return obstacles_points_x, obstacles_points_y

    def get_histogram_grid_active_region(self, active_region_min_x, active_region_min_y, active_region_max_x, active_region_max_y):
        # Return histogram_grid[active_region_min_x:active_region_max_y]
        return [self.histogram_grid[row][active_region_min_x:active_region_max_x + 1] for row in range(active_region_min_y, active_region_max_y + 1)]
        # return [self.histogram_grid[row][active_region_min_y:active_region_max_y + 1] for row in range(active_region_min_x, active_region_max_x + 1)]


def get_discrete_displacement(discrete_start, discrete_end):
    return tuple(map(sub, discrete_start, discrete_end))
