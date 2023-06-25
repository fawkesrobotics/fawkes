
#*****************************************************************************
#                 Makefile Build System for Fawkes: ROS2 bits
#                            -------------------
#   Created on Sunday November 07 12:00:00 2021
#   Copyright (C) 2021 by Gjorgji Nikolovski, Carologistics RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************



ifndef __buildsys_config_mk_
$(error config.mk must be included before ros2.mk)
endif

ifndef __buildsys_ros2_mk_
__buildsys_ros2_mk_ := 1

  ros2-pkg-path = 
  ros2-have-pkgs    = $(if $(strip $(subst 1,,$(foreach p,$1,$(call ros2-have-pkg,$p)))),0,1)
  ros2-pkgs-cflags  = $(foreach p,$1,$(call ros2-pkg-cflags,$p) )
  ros2-pkgs-lflags  = $(foreach p,$1,$(call ros2-pkg-lflags,$p) )
  ros2-missing-pkgs = $(strip $(foreach p,$1,$(if $(subst 1,,$(call ros2-have-pkg,$p)),$p )))

  ros2-have-pkg     = $(shell python3 $(FAWKES_BASEDIR)/etc/buildsys/get_flags.py -p $(1) -e)
  ros2-pkg-cflags   = $(shell python3 $(FAWKES_BASEDIR)/etc/buildsys/get_flags.py  -p $(1) -c)
  ros2-pkg-lflags   = $(subst -l:,,$(shell python3 $(FAWKES_BASEDIR)/etc/buildsys/get_flags.py  -p $(1) -l))
  ros2-pkg-version  = $(shell python3 $(FAWKES_BASEDIR)/etc/buildsys/get_flags.py -p $(1) -v)
  ros2-pkg-version-atleast = $(if $(shell python3 $(FAWKES_BASEDIR)/etc/buildsys/get_flags.py -p $(1) -v; echo $${?/1/}),1,0)

  HAVE_ROS2 = $(call ros2-have-pkg,rclcpp)

  CFLAGS_ROS2  = -DHAVE_ROS2 -DBOOST_BIND_GLOBAL_PLACEHOLDERS $(RCLCPP_cflags) $(call ros2-pkg-cflags,rclcpp)
  LDFLAGS_ROS2 = $(RCLCPP_lflags) $(call ros2-pkg-lflags,rclcpp)

endif # __buildsys_ros2_mk_
