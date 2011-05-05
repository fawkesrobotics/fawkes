#*****************************************************************************
#           Makefile Build System for Fawkes : BaseApp library
#                            -------------------
#   Created on Wed May 04 23:47:31 2011
#   copyright (C) 2006 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

# Check for availability of libdaemon
HAVE_LIBDAEMON = $(if $(shell $(PKGCONFIG) --exists 'libdaemon'; echo $${?/1/}),1,0)
ifeq ($(HAVE_LIBDAEMON),1)
  CFLAGS_LIBDAEMON  += -DHAVE_LIBDAEMON $(shell $(PKGCONFIG) --cflags 'libdaemon')
  LDFLAGS_LIBDAEMON += $(shell $(PKGCONFIG) --libs 'libdaemon')
else
  WARN_TARGETS += warning_libdaemon
endif
