#*****************************************************************************
#            Makefile Build System for Fawkes: Skiller Plugin
#                            -------------------
#   Created on Mon Mar 10 11:12:14 2008
#   Copyright (C) 2006-2008 by Tim Niemueller, AllemaniACs RoboCup Team
#
#   $Id$
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

SKILLDIR = $(abspath $(BASEDIR)/src/plugins/skiller/skills)

ifneq ($(wildcard /usr/include/sys/inotify.h),)
  HAVE_INOTIFY=1
endif

