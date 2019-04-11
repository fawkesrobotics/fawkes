#*****************************************************************************
#            Makefile Build System for Fawkes: MongoDB Plugin
#                            -------------------
#   Created on Sun Dec 05 23:03:18 2010 (Steelers vs. Baltimore)
#   Copyright (C) 2006-2010 by Tim Niemueller, AllemaniACs RoboCup Team
#                 2019      by Till Hofmann
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
	ifeq ($(HAVE_CPP11),1)
    HAVE_MONGODB = $(if $(shell $(PKGCONFIG) --exists --atleast-version=3 'libmongocxx'; echo $${?/1/}),1,0)
    CFLAGS_MONGODB = $(shell $(PKGCONFIG) --cflags 'libmongocxx')
    LDFLAGS_MONGODB = $(shell $(PKGCONFIG) --libs 'libmongocxx')
  endif
endif
