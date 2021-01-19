#! /usr/bin/python

from occupancy_analysis_fct import *
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


def read_hist(filename) :
    pass

def compute_uniform_dist_score(hist) :
    sum = 0
    for line in hist :
        for elt in hist :
            sum += elt
    uni_ref_val = float(sum)/float((len(hist)*len(hist)))
    score = 0;
    for line in hist :
        for elt in hist :
            score += float(elt)/uni_ref_val    
    return score/float(sum)

>>>>>>> Stashed changes
for folder in os.listdir(sys.argv[1]) :
    if folder.split('_')[0] != "maze" and folder.split('_')[0] != "legged" :
        continue
    print(folder)
    for file in os.listdir(sys.argv[1] + "/" + folder) :
        if(file.split('_')[0] == "bd") :
            #print(file)
            filename = str(sys.argv[1]) + "/" + folder + "/"  + file
            hist_filename = str(sys.argv[1]) + "/" + folder + "/" + "grid_" + file
            if(os.path.exists(hist_filename)) :
                print(hist_filename + " already exists !")
                continue
            pointx,pointy = load_data(filename,3)
            hist = convert_to_hist_grid(pointx,pointy,bin=12)
<<<<<<< Updated upstream
            write_hist(hist_filename,hist)

=======
            print(compute_uniform_dist_score(hist))
            write_hist(hist_filename,hist)
>>>>>>> Stashed changes
