##
## CMakeLists.txt
## Copyright (C) 2009 Aldebaran Robotics
##

cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)
#CMAKEMAIN
include("${CMAKE_CURRENT_SOURCE_DIR}/bootstrap.cmake")

use(NAOQI-PLUGINS-TOOLS)
create_module(allaser DEFAULT_REMOTE_OFF)

add_subdirectory(src)
