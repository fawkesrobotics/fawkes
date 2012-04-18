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

# Required OpenRAVE version.
# Currently 0.6.2, so VER=0.6 and all minor than 0.6.2 are filtered out
OPENRAVE_VER = 0.6
OPENRAVE_VER_FILTER = 0.6.0 0.6.1

#Check for OpenRAVE
ifneq ($(PKGCONFIG),)
  PKG_OPENRAVE = 'openrave'
  ifneq ($(shell $(PKGCONFIG) --exists $(PKG_OPENRAVE); echo $${?/1/}),)
    EXISTS_OPENRAVE = 1
  else
    PKG_OPENRAVE ='openrave'$(OPENRAVE_VER)
    EXISTS_OPENRAVE := $(if $(shell $(PKGCONFIG) --exists $(PKG_OPENRAVE); echo $${?/1/}),1,0)
  endif

  HAVE_PYTHON := $(if $(shell $(PKGCONFIG) --exists 'python'; echo $${?/1/}),1,0)
endif

ifeq ($(EXISTS_OPENRAVE),1)
  MODVERSION_OPENRAVE  = $(shell $(PKGCONFIG) --modversion $(PKG_OPENRAVE))

  ifeq ($(if $(filter-out $(OPENRAVE_VER)%,$(MODVERSION_OPENRAVE)),,1)$(if $(filter-out $(OPENRAVE_VER_FILTER),$(MODVERSION_OPENRAVE)),1,),11)
    HAVE_OPENRAVE    = 1
    CFLAGS_OPENRAVE  = $(shell $(PKG_OPENRAVE)-config --cflags-only-I)
    LDFLAGS_OPENRAVE = $(shell $(PKG_OPENRAVE)-config --libs-core)
  endif
endif

ifeq ($(HAVE_PYTHON),1)
  CFLAGS_PYTHON    = $(shell python-config --includes)
  LDFLAGS_PYTHON   = $(shell python-config --libs)
endif
