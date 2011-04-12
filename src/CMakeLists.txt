##
## CMakeLists.txt
## Login : <ctaf@localhost.localdomain>
##
## Author(s):
##  - Cedric GESTES <gestes@aldebaran-robotics.com>
##
## Copyright (C) 2009 Aldebaran Robotics
##

SET(module_SRCS
  allaser.cpp allaser.h
  lasermain.cpp
)

configure_src_module(allaser ${module_SRCS})
use_lib(allaser ALCOMMON RTTOOLS URG )
