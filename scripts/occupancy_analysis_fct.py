#! /usr/bin/python

import scipy.stats as spstat
import numpy.linalg as nplinalg

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
#        print(pointx[i],pointy[i])
#        print(x,y)
        hist[x][y]+=1
    return hist

def write_hist(filename,hist) :
    with open(filename,'w') as f :
        for line in hist :
            for elt in line :
                f.write(str(elt)+" ")
            f.write("\n")


def read_hist(filename) :
    hist = []
    with open(filename) as f :
        for line in f :
            templist = []
            for n in line.split(' ') :
                if(n == '\n') :
                    continue
                templist.append(int(n))
            hist.append(templist)
    return hist        




def JSD(P,Q) :
    P = P / nplinalg.norm(P,ord=1)
    Q = Q / nplinalg.norm(Q,ord=1)
    M = (P + Q)*.5
    return (spstat.entropy(P,M) + spstat.entropy(Q,M))*.5

def compute_uniform_dist_score(hist) :
    hist1D = [0 for _ in range(len(hist)*len(hist))]
    sum = 0
    i = 0
    for line in hist :
        for elt in line :
            sum += elt
            hist1D[i] = elt
            i += 1
    uni_ref_val = 1./float((len(hist)*len(hist)))
    uni_hist = [uni_ref_val for _ in range(len(hist)*len(hist))]

    return 1 - JSD(uni_hist,hist1D)
