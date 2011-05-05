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
#LDFLAGS += -Wl,-R$(PLUGINDIR) -L$(PLUGINDIR) \
#	   $(patsubst %,-l:%,$(subst $(abspath $(PLUGINDIR))/,,$(wildcard $(PLUGINDIR)/*.so))) \
#	   -lpthread

LIBS_fawkes = stdc++ fawkescore fawkesutils fawkesblackboard fawkesnetcomm \
	      fawkesconfig fawkesplugin fawkesaspects fawkesbaseapp
OBJS_fawkes = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))

OBJS_all     = $(OBJS_fawkes)
BINS_all     = $(BINDIR)/fawkes
MANPAGES_all = $(MANDIR)/man8/fawkes.8

include $(BUILDSYSDIR)/base.mk

