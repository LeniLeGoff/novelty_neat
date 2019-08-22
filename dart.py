#!/usr/bin/python
# -*- coding: utf-8 -*-

""" Alexandre Coninx
    ISIR CNRS/UPMC
    29/05/2018
""" 

import numpy as np

import os
from waflib.Configure import conf

def options(opt):
  opt.add_option('--dart', type='string', help='path to Dart', dest='dart')
  opt.add_option('--dart',type='string', help='path to Dart', dest='dart')

@conf
def check_dart(conf):
  # Get locations where to search for ROS's header and lib files
  if conf.options.dart:
    includes_check = [conf.options.dart + '/include']
    libs_check = [conf.options.dart + '/lib']
  else:
    includes_check = ['/usr/include/', '/usr/local/include/']
    libs_check = ['/usr/lib', '/usr/lib/x86_64-linux-gnu/', '/usr/local/lib/']

  try:
    # Find the header for ROS
    conf.start_msg('Checking for Dart includes')
    inclOK = conf.find_file('dart/dart.hpp', includes_check)
    #print inclOK
    conf.end_msg('ok')
    

    # Find the lib files
    libs = ['dart','dart-utils','dart-external-odelcpsolver'] #'dart-optimizer-ipopt', 
    conf.start_msg('Checking for Dart libs')
    for lib in libs:
      libOK = conf.find_file('lib'+lib+'.so', libs_check)
      #print libOK
    conf.end_msg('ok')


    conf.env.INCLUDES_DART = includes_check + [conf.env.INCLUDES_BULLET[0]+"/bullet"] # Fix weird Dart include of Bullet
    conf.env.LIBPATH_DART = libs_check
    conf.env.LIB_DART = libs + ["assimp"]
#    conf.env.DEFINES_DART = ['USE_DART']
  except:
    conf.end_msg('Not found', 'RED')
    return

