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

HAVE_ROSVERSION := $(if $(shell type -p rosversion; echo $${?/1/}),1,0)
HAVE_PKGCONFIG := 

ifeq ($(HAVE_ROSVERSION), 1)
  ROS_VERSION = $(shell rosversion -d)

  ros-pkg-path = $(shell rospack find $(1))
  ros-have-pkgs    = $(if $(strip $(subst 1,,$(foreach p,$1,$(call ros-have-pkg,$p)))),0,1)
  ros-pkgs-cflags  = $(foreach p,$1,$(call ros-pkg-cflags,$p) )
  ros-pkgs-lflags  = $(foreach p,$1,$(call ros-pkg-lflags,$p) )
  ros-missing-pkgs = $(strip $(foreach p,$1,$(if $(subst 1,,$(call ros-have-pkg,$p)),$p )))

  ifeq ($(ROS_VERSION),fuerte)
    ros-have-pkg     = $(if $(shell rospack find $(1) 2>&1 >/dev/null; echo $${?/0/}),0,1)
    ros-pkg-msg-cflags = -I$(call ros-pkg-path,$(1))/msg_gen/cpp/include \
			 -I$(call ros-pkg-path,$(1))/srv_gen/cpp/include
    ros-pkg-cflags   = $(call ros-pkg-msg-cflags,$(1)) \
	$(shell rospack export --lang=cpp --attrib=cflags $(1) 2>/dev/null)
    ros-pkg-lflags   = $(shell rospack export --lang=cpp --attrib=lflags $(1) 2>/dev/null)
    ros-pkg-version  = unknown
    ros-pkg-version-atleast =
  else
    # This has been tested with indigo and hydro, there is no groovy around to test

    ROS_BASE=$(firstword $(foreach p,$(call split,:,$(CMAKE_PREFIX_PATH)),$(patsubst %/.catkin,%,$(wildcard $p/.catkin))))

    ifneq ($(PKGCONFIG),)
      ifeq ($(findstring $(ROS_BASE),$(call split,:,$(PKG_CONFIG_PATH))),)
        # That is essentially what roscd without argument is doing
        export PKG_CONFIG_PATH   := $(ROS_BASE)/lib/pkgconfig$(if $(PKG_CONFIG_PATH),:$(PKG_CONFIG_PATH))
      endif
    endif

    ros-have-pkg     = $(if $(shell $(PKGCONFIG) --exists "$(1)"; echo $${?/1/}),1,0)
    ros-pkg-cflags   = $(shell $(PKGCONFIG) --cflags "$(1)")
    ros-pkg-lflags   = $(subst -l:,,$(shell $(PKGCONFIG) --libs "$(1)"))
    ros-pkg-version  = $(shell $(PKGCONFIG) --modversion "$(1)")
    ros-pkg-version-atleast = $(if $(shell $(PKGCONFIG) --atleast-version "$(2)" "$(1)"; echo $${?/1/}),1,0)
  endif

  HAVE_ROS = $(call ros-have-pkg,roscpp)
else
  HAVE_ROS = 0
endif

ifeq ($(HAVE_ROS),1)
  CFLAGS_ROS  = -DHAVE_ROS $(call ros-pkg-cflags,roscpp)
  LDFLAGS_ROS = $(call ros-pkg-lflags,roscpp)
endif

endif # __buildsys_ros_mk_

