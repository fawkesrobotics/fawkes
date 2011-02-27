#*****************************************************************************
#           Makefile Build System for Fawkes: OpenNI Plugin Config
#                            -------------------
#   Created on Sat Feb 26 15:53:37 2011
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

ifneq ($(wildcard $(SYSROOT)/usr/include/ni/XnCppWrapper.h),)
  HAVE_OPENNI=1
  CFLAGS_OPENNI = -DHAVE_OPENNI -I$(SYSROOT)/usr/include/ni -Wno-unused-variable -Wno-reorder
  LDFLAGS_OPENNI = -lOpenNI
endif

