#! /usr/bin/env python

import os
import sys
import sferes

sys.path.insert(0,sys.path[0] + '/waf_tools')
import dart, bullet

def options(opt):
    opt.load('dart')
    #opt.load('bullet')

def configure(conf):
    conf.load('dart')
    #conf.load('bullet')
    #conf.check_bullet()
    conf.check_dart(required=True)



def build(bld):
  libs = 'EIGEN3 BOOST PTHREAD TBB'  
  cxxflg = bld.get_env()['CXXFLAGS'] 
  print ("Entering directory `" + os.getcwd() + "/modules/'")
 
  bld.program(features = 'cxx',
                source = 'maze.cpp',
                includes = '../../ ../../modules /usr/include/eigen3 /usr/include/ /usr/include/SDL',
                uselib = libs + ' SDL',
                use = 'sferes2 nn2 fastsim',
                target = 'maze')

  sferes.create_variants(bld,
                        source = 'maze.cpp',
                        includes = '../../ ../../modules /home/le_goff/libraries/include/eigen3 /home/le_goff/libraries/include/ /usr/include/ /usr/include/SDL /usr/include/eigen3/',
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
#BIPED  
  # bld.program(features = 'cxx',
  #               source = 'biped/test-dart.cpp',
  #               includes = '../../ /usr/include/ /usr/include/eigen3/ /home/le_goff/libraries/include',
  #               uselib = libs + ' DART',
  #               target = 'test-dart')

  # bld.program(features = 'cxx',
  #               source = 'biped/test_biped.cpp biped/biped.cpp',
  #               includes = '../../ ../../modules /usr/include/ /usr/include/eigen3/ /home/le_goff/libraries/include',
  #               uselib = libs + ' DART DART_GRAPHIC',
  #               defines = 'VISU',
  #               #use = 'sferes2 nn2',
  #               target = 'test_biped')


  # sferes.create_variants(bld,
  #                       source = 'biped/biped_walk.cpp biped/biped.cpp',
  #                       includes = '../../ ../../modules /usr/include/eigen3 /usr/include/ /usr/include/bullet',
  #                       uselib = libs + ' DART DART_GRAPHIC',
  #                       use = 'sferes2 nn2', 
  #                       target = 'biped_walk',
  #                       variants = [
  #                           'NOVELTY NEAT VISU',
  #                           'NOVELTY NEAT',
  #                           'NOVELTY RNN VISU',
  #                           'NOVELTY RNN',
  #                           'NOVELTY VISU',
  #                           'NOVELTY',
  #                           'VISU'
  #                           ])


#LEGGED ROBOT
  bld.program(features = 'cxx',
              source = 'legged_robot/test_simu.cpp legged_robot/legged_robot.cpp',
              includes = '../../ ../../modules /usr/include/ /usr/include/eigen3/ /home/le_goff/libraries/include',
              uselib = libs + ' DART DART_GRAPHIC',
              defines = 'VISU',
              target = 'test_simu')

  sferes.create_variants(bld,
                      source = 'legged_robot/legged_robot_exp.cpp legged_robot/legged_robot.cpp',
                      includes = '../../ ../../modules /usr/include/eigen3 /usr/include/ /usr/include/bullet',
                      uselib = libs + ' DART DART_GRAPHIC',
                      use = 'sferes2 nn2', 
                      target = 'legged_robot',
                      variants = [
                          'NOVELTY NEAT VISU',

                          # 'NOVELTY NEAT THREE_LEGS_2DOF',
                          # 'NOVELTY NEAT THREE_LEGS_3DOF',
                          # 'NOVELTY NEAT THREE_LEGS_4DOF',

                          # 'NOVELTY NEAT THREE_LEGS_2DOF LONG_RUN',
                          # 'NOVELTY NEAT THREE_LEGS_3DOF LONG_RUN',
                          # 'NOVELTY NEAT THREE_LEGS_4DOF LONG_RUN',

                          # 'NOVELTY NEAT THREE_LEGS_2DOF LARGE_POP',
                          # 'NOVELTY NEAT THREE_LEGS_3DOF LARGE_POP',
                          # 'NOVELTY NEAT THREE_LEGS_4DOF LARGE_POP',

                          'NOVELTY NEAT THREE_LEGS_2DOF LONG_EVAL',
                          'NOVELTY NEAT THREE_LEGS_3DOF LONG_EVAL',
                          'NOVELTY NEAT THREE_LEGS_4DOF LONG_EVAL',

                          # 'NOVELTY NEAT THREE_LEGS_2DOF LONG_RUN LONG_EVAL',
                          # 'NOVELTY NEAT THREE_LEGS_3DOF LONG_RUN LONG_EVAL',
                          # 'NOVELTY NEAT THREE_LEGS_4DOF LONG_RUN LONG_EVAL',

                          # 'NOVELTY NEAT THREE_LEGS_2DOF LONG_RUN LARGE_POP',
                          # 'NOVELTY NEAT THREE_LEGS_3DOF LONG_RUN LARGE_POP',
                          # 'NOVELTY NEAT THREE_LEGS_4DOF LONG_RUN LARGE_POP',

                          # 'NOVELTY NEAT THREE_LEGS_2DOF LARGE_POP LONG_EVAL',
                          # 'NOVELTY NEAT THREE_LEGS_3DOF LARGE_POP LONG_EVAL',
                          # 'NOVELTY NEAT THREE_LEGS_4DOF LARGE_POP LONG_EVAL',

                          # 'NOVELTY NEAT THREE_LEGS_2DOF LONG_RUN LONG_EVAL LARGE_POP',
                          # 'NOVELTY NEAT THREE_LEGS_3DOF LONG_RUN LONG_EVAL LARGE_POP',
                          # 'NOVELTY NEAT THREE_LEGS_4DOF LONG_RUN LONG_EVAL LARGE_POP',
                          # 'NOVELTY NEAT FOUR_LEGS_2DOF',
                          # 'NOVELTY NEAT FOUR_LEGS_3DOF',
                          # 'NOVELTY NEAT FOUR_LEGS_4DOF',
                          # 'NOVELTY NEAT SIX_LEGS_2DOF',
                          # 'NOVELTY NEAT ', #HEXAPOD 3DOF
                          # 'NOVELTY NEAT SIX_LEGS_4DOF',

                          # 'NOVELTY RNN VISU',
                          # 'NOVELTY RNN NB_HIDDEN_0', #HEXAPOD 3DOF
                          # 'NOVELTY RNN NB_HIDDEN_2', #HEXAPOD 3DOF
                          # 'NOVELTY RNN NB_HIDDEN_4', #HEXAPOD 3DOF
                          # 'NOVELTY RNN NB_HIDDEN_8', #HEXAPOD 3DOF
                          # 'NOVELTY RNN NB_HIDDEN_16', #HEXAPOD 3DOF
                          # 'NOVELTY RNN THREE_LEGS_2DOF NB_HIDDEN_0',
                          # 'NOVELTY RNN THREE_LEGS_2DOF NB_HIDDEN_2',
                          # 'NOVELTY RNN THREE_LEGS_2DOF NB_HIDDEN_4',
                          # 'NOVELTY RNN THREE_LEGS_2DOF NB_HIDDEN_8',
                          # 'NOVELTY RNN THREE_LEGS_2DOF NB_HIDDEN_16',
                          # 'NOVELTY RNN THREE_LEGS_3DOF NB_HIDDEN_0',
                          # 'NOVELTY RNN THREE_LEGS_3DOF NB_HIDDEN_2',
                          # 'NOVELTY RNN THREE_LEGS_3DOF NB_HIDDEN_4',
                          # 'NOVELTY RNN THREE_LEGS_3DOF NB_HIDDEN_8',
                          # 'NOVELTY RNN THREE_LEGS_3DOF NB_HIDDEN_16',
                          # 'NOVELTY RNN THREE_LEGS_4DOF NB_HIDDEN_0',
                          # 'NOVELTY RNN THREE_LEGS_4DOF NB_HIDDEN_2',
                          # 'NOVELTY RNN THREE_LEGS_4DOF NB_HIDDEN_4',
                          # 'NOVELTY RNN THREE_LEGS_4DOF NB_HIDDEN_8',
                          # 'NOVELTY RNN THREE_LEGS_4DOF NB_HIDDEN_16',
                          # 'NOVELTY RNN FOUR_LEGS_2DOF NB_HIDDEN_0',
                          # 'NOVELTY RNN FOUR_LEGS_2DOF NB_HIDDEN_2',
                          # 'NOVELTY RNN FOUR_LEGS_2DOF NB_HIDDEN_4',
                          # 'NOVELTY RNN FOUR_LEGS_2DOF NB_HIDDEN_8',
                          # 'NOVELTY RNN FOUR_LEGS_2DOF NB_HIDDEN_16',
                          # 'NOVELTY RNN FOUR_LEGS_3DOF NB_HIDDEN_0',
                          # 'NOVELTY RNN FOUR_LEGS_3DOF NB_HIDDEN_2',
                          # 'NOVELTY RNN FOUR_LEGS_3DOF NB_HIDDEN_4',
                          # 'NOVELTY RNN FOUR_LEGS_3DOF NB_HIDDEN_8',
                          # 'NOVELTY RNN FOUR_LEGS_3DOF NB_HIDDEN_16',
                          # 'NOVELTY RNN FOUR_LEGS_4DOF NB_HIDDEN_0',
                          # 'NOVELTY RNN FOUR_LEGS_4DOF NB_HIDDEN_2',
                          # 'NOVELTY RNN FOUR_LEGS_4DOF NB_HIDDEN_4',
                          # 'NOVELTY RNN FOUR_LEGS_4DOF NB_HIDDEN_8',
                          # 'NOVELTY RNN FOUR_LEGS_4DOF NB_HIDDEN_16',
                          # 'NOVELTY RNN SIX_LEGS_2DOF NB_HIDDEN_0',
                          # 'NOVELTY RNN SIX_LEGS_2DOF NB_HIDDEN_2',
                          # 'NOVELTY RNN SIX_LEGS_2DOF NB_HIDDEN_4',
                          # 'NOVELTY RNN SIX_LEGS_2DOF NB_HIDDEN_8',
                          # 'NOVELTY RNN SIX_LEGS_2DOF NB_HIDDEN_16',
                          # 'NOVELTY RNN SIX_LEGS_4DOF NB_HIDDEN_0',
                          # 'NOVELTY RNN SIX_LEGS_4DOF NB_HIDDEN_2',
                          # 'NOVELTY RNN SIX_LEGS_4DOF NB_HIDDEN_4',
                          # 'NOVELTY RNN SIX_LEGS_4DOF NB_HIDDEN_8',
                          # 'NOVELTY RNN SIX_LEGS_4DOF NB_HIDDEN_16',

                          'NOVELTY VISU',                          
                          # 'NOVELTY NB_HIDDEN_0', #HEXAPOD 3DOF
                          # 'NOVELTY NB_HIDDEN_2', #HEXAPOD 3DOF
                          # 'NOVELTY NB_HIDDEN_4', #HEXAPOD 3DOF
                          # 'NOVELTY NB_HIDDEN_8', #HEXAPOD 3DOF
                          # # 'NOVELTY NB_HIDDEN_16', #HEXAPOD 3DOF 
                          # 'NOVELTY THREE_LEGS_2DOF NB_HIDDEN_0',
                          # 'NOVELTY THREE_LEGS_2DOF NB_HIDDEN_2',
                          # 'NOVELTY THREE_LEGS_2DOF NB_HIDDEN_4',
                          # 'NOVELTY THREE_LEGS_2DOF NB_HIDDEN_8',
                          # 'NOVELTY THREE_LEGS_2DOF NB_HIDDEN_16',
                          # 'NOVELTY THREE_LEGS_3DOF NB_HIDDEN_0',
                          # 'NOVELTY THREE_LEGS_3DOF NB_HIDDEN_2',
                          # 'NOVELTY THREE_LEGS_3DOF NB_HIDDEN_4',
                          # 'NOVELTY THREE_LEGS_3DOF NB_HIDDEN_8',
                          # 'NOVELTY THREE_LEGS_3DOF NB_HIDDEN_16',
                          # 'NOVELTY THREE_LEGS_4DOF NB_HIDDEN_0',
                          # 'NOVELTY THREE_LEGS_4DOF NB_HIDDEN_2',
                          # 'NOVELTY THREE_LEGS_4DOF NB_HIDDEN_4',
                          # 'NOVELTY THREE_LEGS_4DOF NB_HIDDEN_8',
                          # 'NOVELTY THREE_LEGS_4DOF NB_HIDDEN_16',
                          # 'NOVELTY FOUR_LEGS_2DOF NB_HIDDEN_0',
                          # 'NOVELTY FOUR_LEGS_2DOF NB_HIDDEN_2',
                          # 'NOVELTY FOUR_LEGS_2DOF NB_HIDDEN_4',
                          # 'NOVELTY FOUR_LEGS_2DOF NB_HIDDEN_8',
                          # 'NOVELTY FOUR_LEGS_2DOF NB_HIDDEN_16',
                          # 'NOVELTY FOUR_LEGS_3DOF NB_HIDDEN_0',
                          # 'NOVELTY FOUR_LEGS_3DOF NB_HIDDEN_2',
                          # 'NOVELTY FOUR_LEGS_3DOF NB_HIDDEN_4',
                          # 'NOVELTY FOUR_LEGS_3DOF NB_HIDDEN_8',
                          # 'NOVELTY FOUR_LEGS_3DOF NB_HIDDEN_16',
                          # 'NOVELTY FOUR_LEGS_4DOF NB_HIDDEN_0',
                          # 'NOVELTY FOUR_LEGS_4DOF NB_HIDDEN_2',
                          # 'NOVELTY FOUR_LEGS_4DOF NB_HIDDEN_4',
                          # 'NOVELTY FOUR_LEGS_4DOF NB_HIDDEN_8',
                          # 'NOVELTY FOUR_LEGS_4DOF NB_HIDDEN_16',
                          # 'NOVELTY SIX_LEGS_2DOF NB_HIDDEN_0',
                          # 'NOVELTY SIX_LEGS_2DOF NB_HIDDEN_2',
                          # 'NOVELTY SIX_LEGS_2DOF NB_HIDDEN_4',
                          # 'NOVELTY SIX_LEGS_2DOF NB_HIDDEN_8',
                          # 'NOVELTY SIX_LEGS_2DOF NB_HIDDEN_16',
                          # 'NOVELTY SIX_LEGS_4DOF NB_HIDDEN_0',
                          # 'NOVELTY SIX_LEGS_4DOF NB_HIDDEN_2',
                          # 'NOVELTY SIX_LEGS_4DOF NB_HIDDEN_4',
                          # 'NOVELTY SIX_LEGS_4DOF NB_HIDDEN_8',
                          # 'NOVELTY SIX_LEGS_4DOF NB_HIDDEN_16',
                          
                          'VISU'
                          ])
