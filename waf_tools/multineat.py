#!/usr/bin/python
#Leni Le Goff - 2021

import os
from waflib.Configure import conf

def options(opt):
  opt.add_option('--multineat', type='string', help='path to MultiNEAT', dest='multineat')

@conf
def check_multineat(conf):
  # Get locations where to search for ROS's header and lib files
  if conf.options.multineat:
    includes_check = [conf.options.multineat + '/include']
    libs_check = [conf.options.multineat + '/lib']
  else:
    includes_check = ['/usr/include/', '/usr/local/include/']
    libs_check = ['/usr/lib', '/usr/lib/x86_64-linux-gnu/', '/usr/local/lib/']

  try:
    # Find the header for ROS
    conf.start_msg('Checking for MultiNEAT includes')
    
    #print fileOK
    conf.end_msg('ok')

    # Find the lib files
    conf.start_msg('Checking for MultiNEAT libs')
    libOK=conf.find_file('libMultiNEAT.so', libs_check)
    print(libOK)
    conf.end_msg('ok')

    conf.env.INCLUDES_MULTINEAT = includes_check
    conf.env.LIBPATH_MULTINEAT = libs_check
    conf.env.LIB_MULTINEAT = ['MultiNEAT']
#    conf.env.DEFINES_BULLET = ['USE_BULLET']
  except:
    conf.end_msg('Not found', 'RED')