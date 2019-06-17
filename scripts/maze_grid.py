#! /usr/bin/python

from occupancy_analysis_fct import *
import os
import sys

for folder in os.listdir(sys.argv[1]) :
    if folder.split('_')[0] != "maze" :
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
            write_hist(hist_filename,hist)

