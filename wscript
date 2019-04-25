#! /usr/bin/env python

import os
import sferes

def build(bld):
  libs = 'EIGEN3 BOOST PTHREAD TBB SDL'  
  cxxflags = bld.get_env()['CXXFLAGS']
  print ("Entering directory `" + os.getcwd() + "/modules/'")

  bld.program(features = 'cxx',
                source = 'maze.cpp',
                includes = '../../ ../../modules /usr/include/eigen3 /usr/include/ /usr/include/SDL',
                uselib = libs,
                use = 'sferes2 nn2 fastsim',
                target = 'maze')

  sferes.create_variants(bld,
                        source = 'maze.cpp',
                        includes = '../../ ../../modules /usr/include/eigen3 /usr/include/ /usr/include/SDL',
                        uselib = libs,
                        use = 'sferes2 nn2 fastsim',
                        target = 'maze',
                        variants = [
                            'NOVELTY NEAT VISU',
                            'NOVELTY NEAT',
                            'NOVELTY RNN VISU',
                            'NOVELTY RNN',
                            'NOVELTY VISU',
                            'NOVELTY',
                            'VISU'
                            ])

  
