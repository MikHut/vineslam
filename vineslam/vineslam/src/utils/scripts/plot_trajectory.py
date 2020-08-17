#!/usr/bin/env python

import glob
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
import argparse


def plotClustersFile(path):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    # ax.axis('equal')

    markers = ['o', '.', 'v', '^', '*', 'X', '1', '2', '3', '4', '5', '6', '7']

    i = 0
    with open(path) as input_file:
        print("Reading %s") % (path)
        for line in input_file:
            line = line.strip()
            numbers = line.split()

            # Centroid
            x_mean, y_mean, z_mean = float(numbers[0]), float(numbers[1]), float(numbers[2])
            ax.scatter(x_mean, y_mean, z_mean, markers[i], s=100)

            # Points
            x = []
            y = []
            z = []
            for idx in range(3, len(numbers), 3):
                x.append(float(numbers[idx + 0]))
                y.append(float(numbers[idx + 1]))
                z.append(float(numbers[idx + 2]))

            ax.scatter(x, y, z, markers[i])
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')

            i += 1

        plt.show()
        i = 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-file", "--path_file", help="File containing robot path.", type=str, required=True)
    args = vars(ap.parse_args(args=arglist))

    path_file = args['path_file']

    plotPath(path_file)


if __name__ == "__main__":
    main()
