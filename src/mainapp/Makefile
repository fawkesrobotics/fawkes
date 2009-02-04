#*****************************************************************************
#            Makefile Build System for Fawkes : Main Application
#                            -------------------
#   Created on Thu Nov 02 19:07:47 2006
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

BASEDIR = ../..

include $(BASEDIR)/etc/buildsys/config.mk
include $(BASEDIR)/src/libs/netcomm/netcomm.mk

# Uncomment the following lines to link the main app against all plugins. This
# makes debugging easier, use it if you see ??? in gdb or valgrind
LDFLAGS += -Wl,-R$(PLUGINDIR) -L$(PLUGINDIR) \
	   $(patsubst %,-l:%,$(subst $(abspath $(PLUGINDIR))/,,$(wildcard $(PLUGINDIR)/*.so))) \
	   -lpthread

# Check for availability of libdaemon
HAVE_LIBDAEMON = $(if $(shell $(PKGCONFIG) --exists 'libdaemon'; echo $${?/1/}),1,0)
ifeq ($(HAVE_LIBDAEMON),1)
  CFLAGS  += -DHAVE_LIBDAEMON $(shell $(PKGCONFIG) --cflags 'libdaemon')
  LDFLAGS += $(shell $(PKGCONFIG) --libs 'libdaemon')
endif

# We need Avahi CFLAGS because of includes in Avahi source, on BSD this is
# in /usr/local/include and thus it needs extra flags
ifeq ($(HAVE_AVAHI),1)
  CFLAGS += $(CFLAGS_AVAHI)
endif

LIBS_fawkes = fawkescore fawkesutils fawkesblackboard fawkesnetcomm fawkesconfig fawkesplugin fawkesaspects
OBJS_fawkes = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))

OBJS_all =	$(OBJS_fawkes)
BINS_all =	$(BINDIR)/fawkes

include $(BASEDIR)/etc/buildsys/base.mk

