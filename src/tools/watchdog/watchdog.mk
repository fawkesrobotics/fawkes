
#*****************************************************************************
#            Makefile Build System for Fawkes : ffwatchdog config
#                            -------------------
#   Created on Thu Mar 31 13:37:37 2011
#   copyright (C) 2011 by Tim Niemueller, AllemaniACs RoboCup Team
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
  CFLAGS  += -DHAVE_LIBDAEMON $(shell $(PKGCONFIG) --cflags 'libdaemon')
  LDFLAGS += $(shell $(PKGCONFIG) --libs 'libdaemon')
else
  WARN_TARGETS += warning_libdaemon
endif
