import yaml
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def load_color_thresholds(yaml_file):
    with open(yaml_file, 'r') as file:
        color_data = yaml.safe_load(file)
    return color_data['orange']

def plot_colors(color_thresholds):
    lower_red = color_thresholds['lower_red']
    upper_red = color_thresholds['upper_red']
    lower_green = color_thresholds['lower_green']
    upper_green = color_thresholds['upper_green']
    lower_blue = color_thresholds['lower_blue']
    upper_blue = color_thresholds['upper_blue']

    # Generate all possible combinations of RGB values
    r_values = np.arange(lower_red, upper_red + 1)
    g_values = np.arange(lower_green, upper_green + 1)
    b_values = np.arange(lower_blue, upper_blue + 1)

    # Create a meshgrid for RGB values
    r_mesh, g_mesh, b_mesh = np.meshgrid(r_values, g_values, b_values, indexing='ij')

    # Flatten the meshgrids
    r_flat = r_mesh.flatten()
    g_flat = g_mesh.flatten()
    b_flat = b_mesh.flatten()

    # Combine RGB values into a single array
    colors = np.column_stack((r_flat, g_flat, b_flat))

    # Plotting
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.scatter(colors[:, 0], colors[:, 1], colors[:, 2], c=colors / 255.0)

    ax.set_xlabel('Red')
    ax.set_ylabel('Green')
    ax.set_zlabel('Blue')
    ax.set_title('Colors within specified range')

    plt.show()

if __name__ == "__main__":
    yaml_file = "colors.yaml"
    color_thresholds = load_color_thresholds(yaml_file)
    plot_colors(color_thresholds)
