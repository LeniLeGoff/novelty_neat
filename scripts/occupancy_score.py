#! /usr/bin/python

from occupancy_analysis_fct import *
import os
import sys

for folder in os.listdir(sys.argv[1]) :
    if folder.split('_')[0] != "maze" and folder.split('_')[0] != "legged" :
        continue
    print(folder)
    if os.path.exists(sys.argv[1] + "/" + folder + "/JSD" + sys.argv[2] + ".dat"):
        print("JSD.dat already exists !")
        continue
    scores = []
    nb_gen = []
    for file in os.listdir(sys.argv[1] + "/" + folder) :
        if(file.split('_')[0] == "grid") :
            if(file.split('_')[1] == sys.argv[2]) :
                nb_gen.append(int(file.split('_')[-1].split('.')[0]))
                filename = str(sys.argv[1]) + "/" + folder + "/"  + file
                hist = read_hist(filename)
                scores.append(compute_uniform_dist_score(hist))
    
    
    nb_gen, scores = zip(*sorted(zip(nb_gen, scores)))
    

    with open(sys.argv[1] + "/" + folder + "/JSD" + sys.argv[2] + ".dat",'w') as jsd_file :
        for i in range(0,len(scores)) :
            jsd_file.write(str(nb_gen[i]) + " " + str(scores[i]) + '\n')

