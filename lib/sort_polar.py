import numpy as np

def sort_polar_array(data):
    """
    Sorts a polar 2D array by wrapped angle, centered around the mean of the highest certainty.

    Args:
        data: A NumPy array or list of lists/tuples, where each element is [index, angle, certainty].

    Returns:
        A NumPy array sorted by wrapped angle.
    """

    data = np.array(data)
    angles = data[:, 0]
    certainties = data[:, 1]

    # Find the indices of the highest certainties
    max_certainty_indices = np.argsort(certainties)[::-1]

    # Calculate the mean angle of the highest certainties (e.g., top 5)
    num_highest = min(7, len(max_certainty_indices))  # Use top 5 or fewer if available
    mean_angle = np.mean(angles[max_certainty_indices[:num_highest]])

    # Calculate the wrapped angles with respect to the mean angle
    wrapped_angles = (angles - mean_angle + 180) % 180 - 360
    # wrapped_angles = (angles - mean_angle + 180) % 360 - 180

    # Sort the data based on the wrapped angles
    sorted_indices = np.argsort(wrapped_angles)
    sorted_data = data[sorted_indices]

    return sorted_data

# Example usage with your provided data:
data = [
    [0.0, 496.6325653328667],
    [10.0, 425.3029327594706],
    [20.0, 318.8823910951653],
    [30.0, 212.4618494308601],
    [40.0, 106.04130776655488],
    [50.0, 17.545454545454547],
    [60.0, 0.0],
    [70.0, 0.0],
    [80.0, 0.0],
    [90.0, 0.0],
    [ 270.0, 0.0],
    [ 280.0, 0.0],
    [ 290.0, 0.0],
    [ 300.0, 0.0],
    [ 310.0, 17.924688443204893],
    [ 320.0, 106.79977556205557],
    [ 330.0, 213.22031722636078],
    [ 340.0, 319.64085889066604],
    [ 350.0, 426.06140055497127],
]

sorted_data = sort_polar_array(data)
print(sorted_data)
