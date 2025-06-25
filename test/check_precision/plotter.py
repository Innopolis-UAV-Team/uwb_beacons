from matplotlib import cm
import matplotlib.pyplot as plt
import matplotlib.colors as clrs
import numpy as np

def plot_on_map(dictionary):
    for dist, value in dictionary.items():
        fig, ax = plt.subplots(figsize=(18, 9))
        plt.grid(color='gray', linestyle='--', linewidth=0.5)

        i = 0
        scatters = []
        colors = ["green", "orange"]

        for anchor_dist, value1 in value.items():
            r = round(float(anchor_dist.split(" ")[1]), 2)
            #draw arc
            arc_angles = np.linspace(0 * np.pi, np.pi, 20)
            arc_xs = r * np.cos(arc_angles)
            arc_ys = r * np.sin(arc_angles)
            plt.plot(arc_xs, arc_ys, linestyle="--", color='blue', lw=1)
            xy = (arc_xs[0], arc_ys[0])
            if i % 2 == 0:
                xy = (arc_xs[-1], arc_ys[0])
            plt.gca().annotate(f'{r} m', xy=xy, xycoords='data', fontsize=8, rotation=10)
            i += 1

            for angle, value2 in value1.items():
                anchor_pose_x = r * np.cos(angle)
                anchor_pose_y = r * np.sin(angle)
                for tag, tag_info in value2.items():
                    tag_n = int(tag.split(' ')[1])
                    dr = tag_info["STD"]
                    rmse = tag_info["RMSE"]
                    est = tag_info["Estimated_val"]
                    angles = np.linspace(0 * np.pi, 2 * np.pi, 100)

                    # tag_pos = (anchor_pose_x + anchor_pose_x*0.05*(-1)**(tag_n%2), anchor_pose_y)

                    # Show std of a tag
                    xs = anchor_pose_x + dr * np.cos(angles)
                    ys = anchor_pose_y + dr * np.sin(angles)
                    plt.plot(xs, ys, color=colors[tag_n])
                    # plt.scatter(anchor_pose_x, anchor_pose_y, c=rmse, cmap='viridis')
                    plt.scatter(anchor_pose_x+tag_n * 0.05, anchor_pose_y, c=rmse, cmap='viridis', vmin=0, vmax=max_err)

                    # Show the results for each tag
                    # est_x = est * np.cos(angle)
                    # est_y = est * np.sin(angle)
                    # plt.scatter(est_x, est_y, c=colors[tag_n])
                    # plt.gca().annotate(f'tag_{tag_n+1}', xy=(est_x, est_y), xycoords='data', fontsize=8, rotation=10)

            txt = f"Circles represents the deviation of a tag: {colors[0]} - first, {colors[1]} - second\n The color of a scatter represents a RMSE of measurement. Left scatter - first, right - second tag"
            fig.text(.5, .05, txt, ha='center')
        cbar = plt.colorbar()
        cbar.ax.set_xlabel('RMSE m', fontsize=20)

        # plt.xlim(-max_dist, max_dist)
        # plt.ylim(-0.5, max_dist)
        plt.show()