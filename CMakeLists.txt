## Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file.

cmake_minimum_required(VERSION 2.8)
project(ALLaser)
find_package(qibuild)

SET(module_SRCS
  src/allaser.cpp
  src/allaser.h
  src/lasermain.cpp
)

qi_create_lib(allaser SHARED SUBFOLDER naoqi ${module_SRCS})
qi_use_lib(allaser ALCOMMON RTTOOLS URG)
