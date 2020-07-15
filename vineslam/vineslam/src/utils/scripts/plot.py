#!/usr/bin/env python

import glob
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np


def plotClustersFile(path):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    # ax.axis('equal')

    markers = ['o', '.', 'v', '^', '*', 'X', '1', '2', '3', '4', '5', '6', '7']

    i = 0
    with open(path) as input_file:
        for line in input_file:
            line = line.strip()
            numbers = line.split()

            x = []
            y = []
            z = []
            for idx in range(1, len(numbers), 3):
                x.append(float(numbers[idx + 0]))
                y.append(float(numbers[idx + 1]))
                z.append(float(numbers[idx + 2]))

            ax.scatter(x, y, z, markers[i])
            ax.set_xlabel('X Label')
            ax.set_ylabel('Y Label')
            ax.set_zlabel('Z Label')

            print(
                '%d %d %d' % (len(x), len(y), len(z)))

            i += 1

        plt.show()
        i = 0


def readClustersFiles(dir):
    return glob.glob(dir + "*.txt")


def main():
    dir = "/home/andre-criis/Source/vineslam_data/"
    files = readClustersFiles(dir)
    for file in files:
        plotClustersFile(file)


if __name__ == "__main__":
    main()
