#!/usr/bin/env python2
'''
@brief it plots the the path by reading the route file
'''

from __future__ import print_function
import yaml
import matplotlib.pyplot as plt

import pdb
import argparse


def readfile(fpath='route.txt'):
    retx = []
    rety = []
    with open(fpath, 'r') as fp:
        for _, point in enumerate(fp):
            # point = fp.readline()
            temp_point = point.split(" ")
            x = float(temp_point[0])
            y = float(temp_point[1])
            retx += [x]
            rety += [y]
    return retx, rety


def main(route_path, ncol=5):
    if ncol < 1:
        ncol = 1
    total_plots = len(route_path)
    if total_plots == 1:
        xarr, yarr = readfile(route_path[0])
        plt.plot(xarr, yarr, '-o')
        plt.title(route_path[0])
        plt.axis('equal')
        plt.show()
        return

    nrow = (total_plots + ncol - 1) / ncol
    # print(len(route_path))
    fig, axs = plt.subplots(nrow, ncol)
    route_counter = 0

    if nrow == 1 or ncol == 1:
        for col in range(total_plots):
            # pdb.set_trace()
            xarr, yarr = readfile(route_path[col])
            axs[col].plot(xarr, yarr, '-o')
            axs[col].set_title(route_path[col])
            axs[col].axis('equal')
    else:
        for row in range(nrow):
            if route_counter == len(route_path):
                break
            for col in range(ncol):
                if route_counter == len(route_path):
                    break
                # pdb.set_trace()
                xarr, yarr = readfile(route_path[route_counter])
                axs[row, col].plot(xarr, yarr, '-o')
                axs[row, col].set_title(route_path[route_counter])
                axs[row, col].axis('equal')
                route_counter += 1
    plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("input", help="input route file containing the route points", nargs='+', default='routes.yaml')
    parser.add_argument("-c", "--columns", help="number of columns in the plot", type=int, default=5)
    args = parser.parse_args()
    main(args.input, args.columns)
