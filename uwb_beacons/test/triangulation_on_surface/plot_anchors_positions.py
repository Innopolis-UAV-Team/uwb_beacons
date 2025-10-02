import argparse
import os
from typing import Dict, Iterable, Tuple
from matplotlib import pyplot as plt
import numpy as np
import seaborn as sns
from scipy.stats import norm


def parse(string: str) -> Tuple[int, float]:
    """
    parse message from serialport
    :param serialport: serial port
    :return: anchor id distance in meters
    """
    string = string.replace("id: ", "").replace(", data:", "")
    id, distance = string.split(' ')
    return int(id), float(distance)

def parse_file(file_name: str) -> Dict:
    """
    parse message from serialport
    :param serialport: serial port
    :return: anchor id distance in meters
    """
    ids = []
    distances = {}
    with open(file_name, 'r') as f:
        strings = f.readlines()
        for s in strings:
            id, distance = parse(s)
            if id not in distances:
                distances[id] = []
            distances[id].append(distance)
    return distances

def plot_distribution(ax, data: dict, real_distance: float, id):
    xmax = max(data)
    xmin = min(data)
    xmin = xmin - 0.1 * xmax
    xmax += 0.1 * xmax
    ax.set_title(f"Distribution of {id}")
    dist_list = data
    mean = np.mean(dist_list)
    std_dev = np.std(dist_list)

    # Create a range of values for the x-axis
    x = np.linspace(min(dist_list) - 3 * std_dev, max(dist_list) + 3 * std_dev, 1000)

    # Calculate the normal distribution's y-values
    y = norm.pdf(x, mean, std_dev)

    # Plot the normal distribution
    ax.plot(x, y, label=f'Anchor (μ={mean:.2f}, σ={std_dev:.2f})')
    ax.axvline(x=real_distance, color='red', label='Real Distance')
    # Fill the region within one standard deviation from the mean
    ax.fill_between(x, y, where=((x > mean - std_dev) & (x < mean + std_dev)), 
                    alpha=0.3, color='gray', label='Standard Deviation')
    ax.set_xlabel('Distance, cm')
    ax.set_ylabel('Probability Density')
    ax.legend()
    ax.grid()
    ax.set_xlim([xmin, xmax])
    return ax

def plot_positions(data, id, ax):
    """
    Plots the distances.

    :param distances: dictionary is list of distances
    :param title: title of the plot
    """
    ax.set_title(f"Position {id}")
    xmax = max(data)
    xmin = min(data)
    xmin = xmin - 0.1 * xmax
    xmax += 0.1 * xmax
    sns.histplot(data, bins=100, color='green', label='Measured position', ax=ax)
    ax.set_xlabel('Distance, cm')
    ax.set_ylabel('Probability Density')
    ax.legend()
    ax.grid()
    ax.set_xlim([xmin, xmax])

def plot_anchor_position(ax, id, x, y):
    pass

def get_hypotenuse(x, y):
    return np.sqrt(x**2 + y**2)

if __name__ == '__main__':
    id_1_pos_x = 500
    id_1_pos_y = 0

    id_2_pos_x = 500
    id_2_pos_y = 500

    id_4_pos_x = 0
    id_4_pos_y = 500

    real_positions = {1: (id_1_pos_x, id_1_pos_y),
                      2: (id_2_pos_x, id_2_pos_y),
                      4: (id_4_pos_x, id_4_pos_y)}

    real_distances = {  1: get_hypotenuse(*real_positions[1]),
                        2: get_hypotenuse(*real_positions[2]),
                        4: get_hypotenuse(*real_positions[4])}

    ids = real_distances.keys()

    directory = os.path.dirname(os.path.realpath(__file__))
    data_directory = os.path.join(directory, 'data')
    files = os.listdir(data_directory)
    for file in files:
        if not ".txt" in file:
            print(file)
            continue
        distaces = parse_file(data_directory + "/" + file)
        fig, axs = plt.subplots(nrows=len(ids), ncols=2, figsize=(10, 10), tight_layout=True)

        # make the subfigure in the empty gridspec slots:
        for i, id in enumerate(ids):
            axs[i][1] = plot_distribution(axs[i][1], distaces[id], real_distances[id], id)
            axs[i][0] = plot_positions(distaces[id], id, axs[i][0])

        fig.savefig(f"{directory}/figs/triangulation_{file.replace('.txt', '')}_.png")
        fig.clear()
