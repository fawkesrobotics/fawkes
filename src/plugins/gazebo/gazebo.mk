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

ifndef __buildsys_config_mk_
$(error config.mk must be included before gazebo.mk)
endif

ifndef __buildsys_gazebo_mk_
__buildsys_gazebo_mk_ := 1


ifneq ($(PKGCONFIG),)
  HAVE_GAZEBO   = $(if $(shell $(PKGCONFIG) --atleast-version=1.0.1 'gazebo'; echo $${?/1/}),1,0)
endif

ifeq ($(HAVE_GAZEBO),1)
  # Gazebo 8 declared several symbols as deprecated but still uses them.
  # Disable the deprecated declarations warning until this is fixed.
  CFLAGS_GAZEBO  = -DHAVE_GAZEBO $(shell $(PKGCONFIG) --cflags 'gazebo') \
                   -Wno-deprecated-declarations
  LDFLAGS_GAZEBO = $(shell $(PKGCONFIG) --libs 'gazebo') -ldl

  # if ffmpeg is installed, gazebo may have been compiled with support for it
  #   # hence check for headers and add the respective include directories
  ifneq ($(wildcard $(SYSROOT)/usr/include/ffmpeg/libavcodec/avcodec.h),)
    CFLAGS_GAZEBO += -I$(SYSROOT)/usr/include/ffmpeg
  endif
  ifneq ($(wildcard $(SYSROOT)/usr/local/include/ffmpeg/libavcodec/avcodec.h),)
    CFLAGS_GAZEBO += -I$(SYSROOT)/usr/local/include/ffmpeg
  endif
endif

endif # __buildsys_gazebo_mk_

