import os
from typing import Dict, List, Tuple

from matplotlib import pyplot as plt
import numpy as np


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
    results = {"Min": [], "Mean": [], "Std": []}
    ids = list(distances.keys())
    bottom = np.zeros(len(ids))

    # Populate results
    for dist in distances.values():
        results["Min"].append(np.min(dist))
        results["Mean"].append(np.mean(dist))
        results["Std"].append(np.std(dist))


    # Plot bars
    for name, data in results.items():
        print(data, ids, name)
        p = ax.bar(ids, data, 0.6, label=name, bottom=bottom)
        bottom += data
        ax.bar_label(p, label_type='center')

    ax.set_title('Distances by ID')
    ax.legend()
    plt.show()


if __name__ == '__main__':
    # list files in the current directory
    the_file_dir = os.path.dirname(os.path.realpath(__file__))
    print(the_file_dir)
    # get data directory 
    data_dir = os.path.join(the_file_dir, 'data')
    print(data_dir)

    files = os.listdir(data_dir)

    # distances = get_distances('data.txt')
    for file in files:
        data = parse_file(os.path.join(data_dir, file))
        plot_distances(data, file)

