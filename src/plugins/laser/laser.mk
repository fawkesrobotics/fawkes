#*****************************************************************************
#            Makefile Build System for Fawkes: Laser Plugin
#                            -------------------
#   Created on Fri Oct 10 00:27:23 2008
#   Copyright (C) 2006-2008 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

ifneq ($(wildcard $(SYSROOT)/usr/include/libpcan.h),)
  HAVE_LIBPCAN=1
  LIBS_LIBPCAN=pcan
  CFLAGS_LIBPCAN = -DHAVE_LIBPCAN
else
  HAVE_LIBPCAN=0
endif

ifneq ($(wildcard $(SYSROOT)/usr/include/urg/UrgCtrl.h),)
  HAVE_URG=1
  LIBS_URG=urg urg_connection urg_monitor urg_coordinate urg_geometry \
	   urg_system urg_common urg_connection_sdl
  CFLAGS_URG = -DHAVE_URG
else
  HAVE_URG=0
endif


HAVE_URG_GBX=$(if $(shell $(PKGCONFIG) --exists 'flexiport' 'hokuyo_aist'; echo $${?/1/}),1,0)
ifeq ($(HAVE_URG_GBX),1)
  CFLAGS_URG_GBX  = -DHAVE_URG_GBX $(shell $(PKGCONFIG) --cflags 'flexiport' 'hokuyo_aist')
  LDFLAGS_URG_GBX = $(shell $(PKGCONFIG) --libs 'flexiport' 'hokuyo_aist')
endif

