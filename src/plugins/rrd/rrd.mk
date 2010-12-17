#*****************************************************************************
#            Makefile Build System for Fawkes: RRD Plugin
#                            -------------------
#   Created on Fri Dec 17 00:33:15 2010
#   Copyright (C) 2006-2010 by Tim Niemueller, AllemaniACs RoboCup Team
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
  HAVE_RRD = $(if $(shell $(PKGCONFIG) --exists 'librrd'; echo $${?/1/}),1,0)
endif

ifeq ($(HAVE_RRD),1)
  CFLAGS_RRD    = -DHAVE_RRD $(shell $(PKGCONFIG) --cflags 'librrd')
  LDFLAGS_RRD   = $(shell $(PKGCONFIG) --libs 'librrd')
endif
