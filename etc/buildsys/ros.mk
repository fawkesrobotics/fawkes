#*****************************************************************************
#                 Makefile Build System for Fawkes: ROS bits
#                            -------------------
#   Created on Thu May 05 16:06:36 2011
#   Copyright (C) 2011 by Tim Niemueller, AllemaniACs RoboCup Team
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
$(error config.mk must be included before ros.mk)
endif

ifndef __buildsys_ros_mk_
__buildsys_ros_mk_ := 1


ros-pkg-path = $(shell rospack find $(1))

ros-pkg-msg-cflags = -I$(call ros-pkg-path,$(1))/msg_gen/cpp/include \
		     -I$(call ros-pkg-path,$(1))/srv_gen/cpp/include
ros-pkg-cflags = $(call ros-pkg-msg-cflags,$(1)) \
	$(shell rospack export --lang=cpp --attrib=cflags $(1) 2>/dev/null)
ros-pkg-lflags = $(shell rospack export --lang=cpp --attrib=lflags $(1) 2>/dev/null)
ros-have-pkg = $(if $(shell rospack find $(1) 2>&1 >/dev/null; echo $${?/0/}),0,1)

HAVE_ROSPACK := $(if $(shell type -p rospack; echo $${?/1/}),1,0)

ifeq ($(HAVE_ROSPACK), 1)
  HAVE_ROS = $(call ros-have-pkg,roscpp)
else
  HAVE_ROS = 0
endif

ifeq ($(HAVE_ROS),1)
  CFLAGS_ROS  = -DHAVE_ROS $(call ros-pkg-cflags,roscpp)
  LDFLAGS_ROS = $(call ros-pkg-lflags,roscpp)
endif

endif # __buildsys_ros_mk_

