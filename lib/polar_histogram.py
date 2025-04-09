"""
Added:
    untracked addition see github commit history

    desired_angle : angle to rotate the polar_histogram around
polar_histogram.py

A polar histogram is defined as follows, assuming bin_width=36
    (num_bins = 360 / 36 = 10):

    index, corresponding_angle, histogram_angle
    0, 0, 123
    1, 36, 0
    2, 72, 30
    ...
    9, 324, 0

(equation: i * bin_width = angle)

However, we only keep index in a flat array for histograms, so we don't natively get/set by angle
but instead translate to and from angle.
"""
# PolarHistogram class creates an object to represent the Polar Histogram
class PolarHistogram:
    def __init__(self, num_bins):
        """
        Creates a Polar Histogram object with the number of bins passed.

        Args:
            num_bins: Number of bins to divide polar space around robot into.
            bin_width: Angular width around robot included in each bin.
            histogram: Array storing the values of the polar histogram.
        """
        self.num_bins = num_bins
        self.bin_width = 360/num_bins
        self._polar_histogram = [0] * num_bins

    def __str__(self):
        string = 'index, angle, certainty\n'
        for i, certainty in enumerate(self._polar_histogram):
            string += str(i) + ' ' + str(i * self.bin_width) + ' ' + str(certainty) + '\n'
        return string


    def wrap(self, bin_index):
        """Helper method for covering out of bounds bin_index."""
        while bin_index < 0:
            bin_index += self.num_bins

        while bin_index >= self.num_bins:
            bin_index -= self.num_bins

        return bin_index

    def get(self, bin_index):
        """custom getter covering cases where bin_index is out of bounds."""
        bin_index = self.wrap(bin_index)

        return self._polar_histogram[bin_index]


    def set(self, bin_index, value):
        """custom setter covering cases where bin_index is out of bounds."""
        bin_index = self.wrap(bin_index)

        self._polar_histogram[bin_index] = value

    def get_bin_index_from_angle(self, angle):
        """Returns index 0 <= i < nBins that corresponds to a. "Wraps" around histogram."""
        while angle < 0:
            angle += 360
        while angle > 360:
            angle -= 360

        return int(angle // self.bin_width)

    def get_middle_angle_of_bin(self, bin_index):
        """Returns the angle in the middle of the bin."""
        bin_index = self.wrap(bin_index)
        return (bin_index + 0.5) * self.bin_width


    # TODO: this is never called, needed?
    def get_certainty_from_angle(self, angle):
        """Returns the value of the histogram for the specified bin."""
        return self.get_certainty(self.get_bin_index_from_angle(angle))


    def add_certainty_to_bin_at_angle(self, angle, delta_certainty):
        """Adds the passed value to the current value of the histogram grid."""
        bin_index = self.get_bin_index_from_angle(angle)
        self._polar_histogram[bin_index] += delta_certainty


    def smooth_histogram(self, l):
        """Smooths the values of the histogram using a moving average with a window of length l."""
        smoothed_histogram = [0] * self.num_bins
        # print(f'smooth_histo, smooth parameter l={l}')
        for k_i in range(self.num_bins):
            # print(k_i)
            # print(f'{[self.get(l_i) for l_i in range(k_i-l+1, k_i+l)]}')

            smoothed_histogram[k_i] = sum([(l - abs(k_i-l_i)) * self.get(l_i) for l_i in range(k_i-l+1, k_i+l)]) / (2*l+1)

        print(f'polar_histogram: smooth_histo result, {smoothed_histogram}')
        # TODO here: rotate the smoothed histogram
        bin_size = 360 // self.num_bins
        desired_angle = 0 # 180
        shift = desired_angle // bin_size

        if 0:
            self._polar_histogram = smoothed_histogram
        else:
            self._polar_histogram = smoothed_histogram[shift:] + smoothed_histogram[:shift]

    # new: shift (wraps) the histogram around a desired angle (ex: target direction)
    def rotate_histogram(self, desired_angle=90):
        bin_size = 360 // len(self._polar_histogram)
        shift = desired_angle // bin_size
        return self._polar_histogram[shift:] + self._polar_histogram[:shift]

    def get_angle_certainty_orig(self):
        """Instead of (bin_index, certainty), return (angle, certainty) pairs."""
        return [(i * self.bin_width, certainty) for i, certainty in enumerate(self._polar_histogram)]

    def get_angle_certainty(self):
        """Instead of (bin_index, certainty), return (angle, certainty) pairs."""
        shifted_polar_histo = self.rotate_histogram(0)
        # shifted_polar_histo = self.rotate_histogram(180)
        # shifted_polar_histo = self.rotate_histogram(self._polar_histogram,90)
        return [(i * self.bin_width, certainty) for i, certainty in enumerate(shifted_polar_histo)]
        # return [(i * self.bin_width, certainty) for i, certainty in enumerate(self._polar_histogram)]

    def reset(self):
        self._polar_histogram = [0] * self.num_bins
