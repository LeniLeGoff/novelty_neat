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
  opt.add_option('--bullet', type='string', help='path to Bullet', dest='bullet')

@conf
def check_bullet(conf):
  # Get locations where to search for ROS's header and lib files
  if conf.options.bullet:
    includes_check = [conf.options.bullet + '/include']
    libs_check = [conf.options.bullet + '/lib']
  else:
    includes_check = ['/usr/include/', '/usr/local/include/']
    libs_check = ['/usr/lib', '/usr/lib/x86_64-linux-gnu/', '/usr/local/lib/']

  try:
    # Find the header for ROS
    conf.start_msg('Checking for Bullet includes')
    fileOK = conf.find_file('bullet/btBulletCollisionCommon.h', includes_check)
    #print fileOK
    conf.end_msg('ok')

    # Find the lib files
    libs = ['BulletSoftBody', 'BulletDynamics', 'BulletCollision', 'LinearMath']
    conf.start_msg('Checking for Bullet libs')
    for lib in libs:
      libOK=conf.find_file('lib'+lib+'.so', libs_check)
      #print libOK
    conf.end_msg('ok')

    conf.env.INCLUDES_BULLET = includes_check
    conf.env.LIBPATH_BULLET = libs_check
    conf.env.LIB_BULLET = libs
#    conf.env.DEFINES_BULLET = ['USE_BULLET']
  except:
    conf.end_msg('Not found', 'RED')
    return

