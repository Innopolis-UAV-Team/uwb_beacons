import os
from typing import Dict, List, Tuple
import seaborn as sns
from matplotlib import pyplot as plt
import matplotlib.colors as mcolors

import numpy as np
import pandas as pd

from parce_data import parse_file

colors = list(mcolors.BASE_COLORS.values())

color_map = {}
labels = []

def plot_real_and_measured(real_distance: float, distances: List[float], fig, ax, i: int):
    max_dist = max(distances)
    min_dist = min(distances)
    mean = np.mean(distances)

    norm_dist = 1 - abs(mean - distances) / (max_dist - min_dist)
    print(f"Mean: {mean}, distances: {distances}, norm_dist: {norm_dist}")
    return ax.scatter([real_distance]*len(distances), distances, s=10, color=color_map[i][0], alpha=norm_dist)

def plot_boxplot(ax, distances, ids, color_map):
    sns.boxplot(data=distances, x='real', y='measured', ax=ax)
    return ax


def parse_files(directory: str) -> Dict:

    files = os.listdir(directory)
    ids = []
    for file in files:
        if not ".txt" in file:
            print(file)
            continue
        data = parse_file(os.path.join(directory, file))
        for key in data.keys():
            if key not in ids:
                ids.append(key)
    ids = sorted(ids)
    id_ax_map = {}
    # make plots for each id
    fig, axs = plt.subplots(nrows=len(ids), ncols=1, figsize=(10, 10),tight_layout=True)
    fig.suptitle("Single Anchor Test")
    if not isinstance(axs, list):
        ids = [ids]
        axs = [axs]

    for ax, id in zip(axs, ids):
        ax.set_title(f"id {id}")
        ax.set_xlabel("Real distance, m")
        ax.set_ylabel("Measured distance, m")
        ax.grid(True)
        if isinstance(id, list):
            id = id[0]
        id_ax_map[id] = ax
        if id not in color_map:
            color_map[id] = (colors[len(color_map)], f"id {id}")

    vals = []

    for file in files:
        if not ".txt" in file:
            print(file)
            continue
        data = parse_file(os.path.join(directory, file))
        real_distance = int(file.split('_')[0]) / 100
        for id, distance in data.items():
            for dist in distance:
                vals.append({"id": id, "real": real_distance, "measured": dist / 100})
    df = pd.DataFrame(vals)

    for ax, id in zip(axs, ids):
        if isinstance(id, list):
            id = id[0]
        data = df.loc[df['id'] == id]
        plot_boxplot(ax=ax, distances=data, ids=id, color_map=color_map)

    labels = [x[1] for x in color_map.values()]
    plt.grid(axis='x')
    plt.xlabel("Real distance, m")
    plt.ylabel("Measured distance, m")
    plt.show()
    fig.savefig(f"{directory}/figs/single_anchor_test.png")

if __name__ == "__main__":
    file_dir = os.path.dirname(os.path.realpath(__file__))
    full_path = os.path.join(file_dir, "data/single_anchor_test")
    # check if figs directory exists
    if not os.path.exists(f"{full_path}/figs"):
        os.mkdir(f"{full_path}/figs")
    print(f"Saving figures to {full_path}")
    parse_files(full_path)
