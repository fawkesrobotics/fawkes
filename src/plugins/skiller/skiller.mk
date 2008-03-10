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
LUA_MINVERSION = 5.1

# Check for Lua (Fedora packages lua and lua-devel)
ifneq ($(PKGCONFIG),)
  HAVE_LUA = $(if $(shell $(PKGCONFIG) --atleast-version $(LUA_MINVERSION) 'lua'; echo $${?/1/}),1,0)
endif

ifeq ($(HAVE_LUA),1)
  CFLAGS_LUA = $(shell $(PKGCONFIG) --cflags 'lua')
  LDFLAGS_LUA = $(shell $(PKGCONFIG) --libs 'lua')
endif

