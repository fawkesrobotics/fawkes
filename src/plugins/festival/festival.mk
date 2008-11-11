#*****************************************************************************
#           Makefile Build System for Fawkes: Flite Plugin Config
#                            -------------------
#   Created on Tue Oct 28 14:50:59 2008
#   Copyright (C) 2008 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

ifneq ($(wildcard $(SYSROOT)/usr/include/festival/festival.h),)
  HAVE_FESTIVAL   = 1
  CFLAGS_FESTIVAL = -I/usr/include/speech_tools
  LIBS_FESTIVAL   = Festival estbase estools eststring
endif
