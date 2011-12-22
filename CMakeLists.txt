##
## CMakeLists.txt
## Login : <ctaf@localhost.localdomain>
##
## Author(s):
##  - Cedric GESTES <gestes@aldebaran-robotics.com>
##
## Copyright (C) 2009 Aldebaran Robotics
##

project(ALLaser)
cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)

include("${CMAKE_CURRENT_SOURCE_DIR}/bootstrap.cmake")

SET(module_SRCS
  allaser.cpp allaser.h
  lasermain.cpp
)

qi_create_lib(allaser SHARED SUBFOLDER naoqi ${module_SRCS})
qi_use_lib(allaser ALCOMMON RTTOOLS URG)
