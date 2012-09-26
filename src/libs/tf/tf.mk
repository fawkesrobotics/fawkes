#*****************************************************************************
#        Makefile Build System for Fawkes: Transforms Library Config
#                            -------------------
#   Created on Tue Oct 18 16:23:47 2011
#   Copyright (C) 2011 by Tim Niemueller, AllemaniACs RoboCup Team
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

include $(BUILDSYSDIR)/ros.mk

# if we can find a ROS bullet version we use that version. It is patched
# and required for the ROS integration. When mixing the system and ROS
# bullet version we will get a segfault when shutting down fawkes, always.
USE_ROS_BULLET=0
ifeq ($(HAVE_ROS),1)
  ifeq ($(call ros-have-pkg,bullet),1)
    USE_ROS_BULLET=1
  endif
endif

ifeq ($(USE_ROS_BULLET),1)
  HAVE_BULLET=1
  CFLAGS_BULLET  = $(call ros-pkg-cflags,bullet) -Wno-deprecated-declarations
  LDFLAGS_BULLET = $(call ros-pkg-lflags,bullet)
else
  ifneq ($(PKGCONFIG),)
    HAVE_BULLET = $(if $(shell $(PKGCONFIG) --exists 'bullet'; echo $${?/1/}),1,0)
    ifeq ($(HAVE_BULLET),1)
      CFLAGS_BULLET  = $(shell $(PKGCONFIG) --cflags 'bullet')
      # we're only interested in the math part
      LDFLAGS_BULLET = -lLinearMath
    endif
  endif
endif

ifeq ($(HAVE_BULLET),1)
  HAVE_TF = 1
  CFLAGS_TF  = -DHAVE_TF $(CFLAGS_BULLET) \
	       -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX
  LDFLAGS_TF = $(LDFLAGS_BULLET) -lm
  ifeq ($(HAVE_CPP11),1)
    CFLAGS_TF += $(CFLAGS_CPP11)
  endif
endif
