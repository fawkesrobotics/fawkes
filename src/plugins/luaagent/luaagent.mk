#*****************************************************************************
#            Makefile Build System for Fawkes: LuaAgent Plugin
#                            -------------------
#   Created on Thu Jan 01 14:11:11 2009
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

ifneq ($(wildcard /usr/include/sys/inotify.h),)
  HAVE_INOTIFY=1
endif

