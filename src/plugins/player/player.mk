#*****************************************************************************
#           Makefile Build System for Fawkes: Player Plugin Config
#                            -------------------
#   Created on Mon Sep 29 11:48:13 2008
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

ifneq ($(PKGCONFIG),)
  HAVE_PLAYERCC = $(if $(shell $(PKGCONFIG) --exists 'playerc++'; echo $${?/1/}),1,0)
endif

ifeq ($(HAVE_PLAYERCC),1)
  CFLAGS_PLAYERCC  = $(shell $(PKGCONFIG) --cflags 'playerc++')
  LDFLAGS_PLAYERCC = $(shell $(PKGCONFIG) --libs 'playerc++')
endif

