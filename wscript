#! /usr/bin/env python

import os
import sferes
import dart,bullet

def options(opt):
    opt.load('dart')
    opt.load('bullet')

def configure(conf):
    conf.load('dart')
    conf.load('bullet')
    conf.check_bullet()
    conf.check_dart()



def build(bld):
  libs = 'EIGEN3 BOOST PTHREAD TBB'  
  cxxflags = bld.get_env()['CXXFLAGS']
  print ("Entering directory `" + os.getcwd() + "/modules/'")

  bld.program(features = 'cxx',
                source = 'maze.cpp',
                includes = '../../ ../../modules /usr/include/eigen3 /usr/include/ /usr/include/SDL',
                uselib = libs + ' SDL',
                use = 'sferes2 nn2 fastsim',
                target = 'maze')

  sferes.create_variants(bld,
                        source = 'maze.cpp',
                        includes = '../../ ../../modules /usr/include/eigen3 /usr/include/ /usr/include/SDL',
                        uselib = libs + ' SDL',
                        use = 'sferes2 nn2 fastsim',
                        target = 'maze',
                        variants = [
                            'NOVELTY NEAT VISU',
                            'NOVELTY NEAT',
                            'NOVELTY RNN VISU',
                            'NOVELTY RNN NB_HIDDEN_0',
                            'NOVELTY RNN NB_HIDDEN_2',
                            'NOVELTY RNN NB_HIDDEN_4',
                            'NOVELTY RNN NB_HIDDEN_8',
                            'NOVELTY RNN NB_HIDDEN_16',
                            'NOVELTY VISU',
                            'NOVELTY',
                            'NOVELTY NB_HIDDEN_0',
                            'NOVELTY NB_HIDDEN_2',
                            'NOVELTY NB_HIDDEN_4',
                            'NOVELTY NB_HIDDEN_8',
                            'NOVELTY NB_HIDDEN_16',
                            'VISU'
                            ])
'''  
  bld.program(features = 'cxx',
                source = 'biped/test-dart.cpp',
                includes = '../../ /usr/include/ /usr/include/eigen3/',
                uselib = libs + ' DART',
                target = 'test-dart')


  sferes.create_variants(bld,
                        source = 'biped/biped_walk.cpp biped/biped.cpp',
                        includes = '../../ ../../modules /usr/include/eigen3 /usr/include/',
                        uselib = libs + ' DART BULLET',
                        use = 'sferes2 nn2 fastsim',
                        target = 'biped_walk',
                        variants = [
                            'NOVELTY NEAT VISU',
                            'NOVELTY NEAT',
                            'NOVELTY RNN VISU',
                            'NOVELTY RNN',
                            'NOVELTY VISU',
                            'NOVELTY',
                            'VISU'
                            ])
'''
  
