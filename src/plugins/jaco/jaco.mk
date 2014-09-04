#*****************************************************************************
#              Makefile Build System for Fawkes: Kinova Jaco Plugin
#                            -------------------
#   Created on Tue Jun 04 13:13:20 2013
#   Copyright (C) 2013 by Bahram Maleki-Fard, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

#Check for libkindrv
ifneq ($(PKGCONFIG),)
  HAVE_LIBKINDRV = $(if $(shell $(PKGCONFIG) --exists 'libkindrv'; echo $${?/1/}),1,0)
endif
ifeq ($(HAVE_LIBKINDRV), 1)
  CFLAGS_LIBKINDRV  = $(shell $(PKGCONFIG) --cflags 'libkindrv')
  LDFLAGS_LIBKINDRV = $(shell $(PKGCONFIG) --libs 'libkindrv')
endif
