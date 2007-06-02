#*****************************************************************************
#                 Makefile Build System for Fawkes: Example Plugin
#                            -------------------
#   Created on Wed Nov 22 17:05:17 2006
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
#
#           $Id$
# last modified: $Date$
#            by: $Author$
#
#*****************************************************************************

BASEDIR = ../../..

CFLAGS = -g

LIBS_example = core utils netcomm
OBJS_example = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))

LIBS_example_plugin_netping = core utils netcomm
OBJS_example_plugin_netping = example_plugin_netping.o

OBJS_all    = $(OBJS_example)		\
	      $(OBJS_example_plugin_netping)
BINS_all    = $(BINDIR)/example_plugin_netping
PLUGINS_all = $(PLUGINDIR)/example.so

include $(BASEDIR)/etc/buildsys/base.mk
