#*****************************************************************************
#              Makefile Build System for Fawkes: OpenRAVE Plugin
#                            -------------------
#   Created on Fri Feb 25 15:08:00 2011
#   Copyright (C) 2011 by Bahram Maleki-Fard, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

#Check for openRAVE
ifneq ($(PKGCONFIG),)
  HAVE_OPENRAVE := $(if $(shell $(PKGCONFIG) --exists 'openrave'; echo $${?/1/}),1,0)
endif

ifeq ($(HAVE_OPENRAVE),1)
  CFLAGS_OPENRAVE    = $(shell $(PKGCONFIG) --cflags 'openrave')
  LDFLAGS_OPENRAVE   = $(shell $(PKGCONFIG) --libs 'openrave') -lopenrave-core -lboost_thread-mt
endif
