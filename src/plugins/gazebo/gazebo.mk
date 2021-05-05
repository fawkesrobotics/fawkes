#*****************************************************************************
#                 Makefile Build System for Fawkes: Gazebo bits
#                            -------------------
#   Created on Fri Aug 24 09:32:21 2012
#   Author  Bastian Klingen
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

include $(BUILDSYSDIR)/boost.mk

ifndef __buildsys_config_mk_
$(error config.mk must be included before gazebo.mk)
endif

ifndef __buildsys_gazebo_mk_
__buildsys_gazebo_mk_ := 1


ifneq ($(PKGCONFIG),)
  HAVE_GAZEBO   = $(if $(shell $(PKGCONFIG) --atleast-version=1.0.1 'gazebo'; echo $${?/1/}),1,0)
  HAVE_GAZEBO_96 = $(if $(shell $(PKGCONFIG) --atleast-version=9.6.0 'gazebo'; echo $${?/1/}),1,0)
  HAVE_GAZEBO_10 = $(if $(shell $(PKGCONFIG) --atleast-version=10 'gazebo'; echo $${?/1/}),1,0)
  HAVE_GAZEBO_11 = $(if $(shell $(PKGCONFIG) --atleast-version=11 'gazebo'; echo $${?/1/}),1,0)
  HAVE_GAZEBO_12 = $(if $(shell $(PKGCONFIG) --atleast-version=12 'gazebo'; echo $${?/1/}),1,0)
endif

ifeq ($(HAVE_GAZEBO),1)
	# -Wno-range-loop-construct: gazebo uses range-based for loops without
	#  references in its headers. Disable the warning until this is fixed
	#  upstream.
  CFLAGS_GAZEBO  = -DHAVE_GAZEBO $(shell $(PKGCONFIG) --cflags 'gazebo') \
                   -DTBB_SUPPRESS_DEPRECATED_MESSAGES
  ifneq ($(HAVE_GAZEBO_10),1)
    # Gazebo 8 declared several symbols as deprecated but still uses them.
		# Those were fixed in Gazebo 10.
    # Disable the deprecated declarations warning with older gazebo versions.
    CFLAGS += -Wno-deprecated-declarations
  endif
  LDFLAGS_GAZEBO = $(shell $(PKGCONFIG) --libs 'gazebo') -ldl -lm

  ifeq ($(HAVE_GAZEBO_96)$(boost-have-lib system),11)
    LDFLAGS_GAZEBO += $(boost-lib-ldflags system)
  endif
	# Assume that this is no longer necessary in Gazebo 11.
  ifneq ($(HAVE_GAZEBO_11),1)
    CFLAGS_GAZEBO += -DBOOST_BIND_GLOBAL_PLACEHOLDERS
  endif
  # Assume that this is no longer necessary in Gazebo 12.
  ifneq ($(HAVE_GAZEBO_12),1)
    CFLAGS_GAZEBO += -Wno-range-loop-construct
  endif
  # if ffmpeg is installed, gazebo may have been compiled with support for it
  # hence check for headers and add the respective include directories
  ifneq ($(wildcard $(SYSROOT)/usr/include/ffmpeg/libavcodec/avcodec.h),)
    CFLAGS_GAZEBO += -I$(SYSROOT)/usr/include/ffmpeg
  endif
  ifneq ($(wildcard $(SYSROOT)/usr/local/include/ffmpeg/libavcodec/avcodec.h),)
    CFLAGS_GAZEBO += -I$(SYSROOT)/usr/local/include/ffmpeg
  endif
endif

endif # __buildsys_gazebo_mk_

