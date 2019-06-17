#! /usr/bin/python

import os
import sys

def load_data(filename, pos = 0) :
    pointx = []
    pointy = []
    with open(filename) as f :
        for l in f.readlines():
            l=l.rstrip()
            lc=list(map(float, l.split(" ")))
            pointx.append(lc[0+pos])
            pointy.append(lc[1+pos])
    return pointx, pointy


def convert_to_hist_grid(pointx,pointy,bin = 6) :
    hist = [[0] * bin for _ in range(bin)]
    for i in range(0,len(pointx)) :
        x = int(pointx[i]/600.*bin)
        y = int(pointy[i]/600.*bin)
        if(pointx[i] >= 600.) :
            x = bin - 1 
        if(pointy[i] >= 600.) : 
            y = bin - 1
        print(pointx[i],pointy[i])
        print(x,y)
        hist[x][y]+=1
    
    return hist

def write_hist(filename,hist) :
    with open(filename,'w') as f :
        for line in hist :
            for elt in line :
                f.write(str(elt)+" ")
            f.write("\n")


for folder in os.listdir(sys.argv[1]) :
    if not os.path.isdir(folder) :
        continue
    for file in os.listdir(folder) :
        if(file.split('_')[0] == "bd") :
            print(file)
            filename = str(sys.argv[1]) + file
            hist_filename = str(sys.argv[1]) + "grid_" + file
            pointx,pointy = load_data(filename,3)
            hist = convert_to_hist_grid(pointx,pointy,bin=12)
            write_hist(hist_filename,hist)