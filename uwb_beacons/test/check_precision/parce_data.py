import os
import time
from typing import Dict, Iterable, List, Tuple

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

def get_rmse(distances):
    """
    Calculates the root mean square error (RMSE) of the distances.

    :param distances: dictionary is list of distances
    :return: root mean square error (RMSE) of the distances
    """
    return np.sqrt(np.mean(np.square(distances)))

def get_mean(distances):
    """
    Calculates the mean of the distances.

    :param distances: dictionary is list of distances
    :return: mean of the distances
    """
    return np.mean(distances)

def get_median(distances):
    """
    Calculates the median of the distances.

    :param distances: dictionary is list of distances
    :return: median of the distances
    """
    return np.median(distances)

def get_std(distances):
    """
    Calculates the standard deviation of the distances.

    :param distances: dictionary is list of distances
    :return: standard deviation of the distances
    """
    return np.std(distances)

def get_min(distances):
    """
    Calculates the minimum of the distances.

    :param distances: dictionary is list of distances
    :return: minimum of the distances
    """
    return np.min(distances)

def get_max(distances):
    """
    Calculates the maximum of the distances.

    :param distances: dictionary is list of distances
    :return: maximum of the distances
    """
    return np.max(distances)

def plot_distances(distances, title):
    """
    Plots the distances.

    :param distances: dictionary is list of distances
    :param title: title of the plot
    """
    # fig, ax = plt.subplots()

    # results = {"Min": [], "Mean": [], "Std": []}
    # ids = list(distances.keys())
    # bottom = np.zeros(len(results.keys()))
    # for id , dist in distances.items():
    #     results["Min"].append(np.min(dist))
    #     results["Mean"].append(np.mean(dist))
    #     results["Std"].append(np.std(dist))

    # for name, data in results.items():
    #     print(data, ids, name)
    #     p = ax.bar(ids, data, 0.6, label=name, bottom=bottom)
    #     bottom += data

    #     ax.bar_label(p, label_type='center')
    # ax.set_title('Number of penguins by sex')
    # ax.legend()

    # plt.show()

    fig, ax = plt.subplots()
    results = {"Min": [], "Mean": [], "Std": [], "Max": []}
    ids = list(distances.keys())
    bottom = np.zeros(len(ids))

    # Populate results
    for dist in distances.values():
        results["Min"].append(np.min(dist))
        results["Mean"].append(np.mean(dist))
        results["Std"].append(np.std(dist))
        results["Max"].append(np.max(dist))

    # Plot bars
    for name, data in results.items():
        print(data, ids, name)
        p = ax.bar(ids, data, 0.6, label=name, bottom=bottom)
        bottom += data
        ax.bar_label(p, label_type='center')

    ax.set_title('Distances by ID')
    ax.legend()
    plt.show()

def plot_distribution(data: dict, real_distance: float):
    ids = list(data.keys())
    fig, axs = plt.subplots(nrows=len(ids), ncols=1, figsize=(10, 10),tight_layout=True)
    fig.suptitle("Single Anchor Test")
    if not isinstance(axs, Iterable):
        axs = [axs]
    xmax = max([max(d) for d in data.values()])
    xmin = min([min(d) for d in data.values()])
    xmin = xmin - 0.1 * xmax
    xmax += 0.1 * xmax
    for i, id in enumerate(ids):
        axs[i].set_title(f"id {id}")
        dist_list = data[id]
        mean = np.mean(dist_list)
        std_dev = np.std(dist_list)

        # Create a range of values for the x-axis
        x = np.linspace(min(dist_list) - 3 * std_dev, max(dist_list) + 3 * std_dev, 1000)

        # Calculate the normal distribution's y-values
        y = norm.pdf(x, mean, std_dev)
        sns.histplot(data[id], bins=100, color='green', label='values', ax=axs[i])
        # Plot the normal distribution
        axs[i].plot(x, y, label=f'Anchor {id} (μ={mean:.2f}, σ={std_dev:.2f})')
        axs[i].axvline(x=real_distance, color='red', label='Real Distance')
        # Fill the region within one standard deviation from the mean
        axs[i].fill_between(x, y, where=((x > mean - std_dev) & (x < mean + std_dev)), 
                        alpha=0.3, color='gray', label='Standard Deviation')
        axs[i].set_xlabel('Distance, cm')
        axs[i].set_ylabel('Probability Density')
        axs[i].legend()
        axs[i].grid()
        axs[i].set_xlim([xmin, xmax])

    time_str = time.strftime("%Y-%m-%d_%H-%M-%S")
    fig.savefig(f"{data_dir}/figs/single_anchor_test_{real_distance}_{time_str}.png")

if __name__ == '__main__':
    # list files in the current directory
    the_file_dir = os.path.dirname(os.path.realpath(__file__))
    print(the_file_dir)
    # get data directory 
    data_dir = os.path.join(the_file_dir, 'data')
    print(data_dir)

    files = os.listdir(data_dir)

    for file in files:
        if (".txt" in file):
            data = parse_file(os.path.join(data_dir, file))
            real_distance = int(file.split('_')[0])
            print(real_distance)
            plot_distribution(data, real_distance)
