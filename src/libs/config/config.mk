#*****************************************************************************
#               Makefile Build System for Fawkes: Config Library
#                            -------------------
#   Created on Tue Jun 21 15:04:39 2007
#   copyright (C) 2006-2007 by Tim Niemueller, AllemaniACs RoboCup Team
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
  HAVE_SQLITE    := $(if $(shell $(PKGCONFIG) --exists 'sqlite3'; echo $${?/1/}),1,0)
endif
ifeq ($(HAVE_SQLITE),1)
  CFLAGS += -DHAVE_SQLITE
endif

