#*****************************************************************************
#            Makefile Build System for Fawkes: Roomba Plugin
#                            -------------------
#   Created on Fri Jan 07 23:44:11 2011
#   Copyright (C) 2006-2011 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

HAVE_BLUEZ=$(if $(shell $(PKGCONFIG) --exists 'bluez'; echo $${?/1/}),1,0)
ifeq ($(HAVE_BLUEZ),1)
  CFLAGS_BLUEZ  = -DHAVE_BLUEZ$(shell $(PKGCONFIG) --cflags 'bluez')
  LDFLAGS_BLUEZ = $(shell $(PKGCONFIG) --libs 'bluez')
endif

