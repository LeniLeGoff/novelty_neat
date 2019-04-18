#! /usr/bin/env python

import os
import sferes

def build(bld):
  libs = 'EIGEN3 BOOST PTHREAD TBB SDL'  
  cxxflags = bld.get_env()['CXXFLAGS']
  print ("Entering directory `" + os.getcwd() + "/modules/'")

   
  sferes.create_variants(bld,
                        source = 'maze.cpp',
                        includes = '../../ ../../modules /usr/include/eigen3 /usr/include/ /usr/include/SDL',
                        uselib = libs,
                        use = 'sferes2 nn2 fastsim',
                        target = 'maze',
                        variants = [
                            'NOVELTY NEAT VISU',
                            'NOVELTY NEAT',
                            'NEAT VISU',
                            'NEAT',
                            'NOVELTY VISU',
                            'NOVELTY',
                            'VISU'
                            ])
