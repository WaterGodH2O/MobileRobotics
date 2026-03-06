#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt

def visualization():
    fig, ax = plt.subplots(1)
    ax.set_aspect('equal')

    trajectory = np.loadtxt("./trajectory.csv", delimiter=',')

    ax.plot(trajectory[:, 0], trajectory[:, 1], linewidth=2)
    ax.set_xlim(-1, 5)
    ax.set_ylim(-1, 5)

    plt.show()

if __name__ == "__main__":
    visualization()