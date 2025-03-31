#  this just prints the map.txt
from lib.histogram_grid import HistogramGrid

map_fname = 'map.txt'
resolution = 1 # node size = 1cm
active_region_dimension = (16, 8)
init_location = (1, 1)
# hg = HistogramGrid.build_histogram_from_txt(map_fname, resolution)
hg = HistogramGrid.from_map(map_fname, active_region_dimension, resolution, init_location)
print(*hg.histogram_grid, sep="\n")
