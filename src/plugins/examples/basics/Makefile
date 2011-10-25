#*****************************************************************************
#                 Makefile Build System for Fawkes: Example Plugin
#                            -------------------
#   Created on Wed Nov 22 17:05:17 2006
#   Copyright (C) 2006-2008 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

BASEDIR = ../../../..
include $(BASEDIR)/etc/buildsys/config.mk

LIBS_example = fawkescore fawkesaspects fawkesnetcomm fawkesblackboard TestInterface
OBJS_example = $(filter-out example_plugin_netping.o,$(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp))))))

LIBS_example_plugin_netping = fawkescore fawkesutils fawkesnetcomm
OBJS_example_plugin_netping = example_plugin_netping.o

OBJS_all    = $(OBJS_example)		\
	      $(OBJS_example_plugin_netping)
BINS_all    = $(BINDIR)/example_plugin_netping
PLUGINS_all = $(PLUGINDIR)/example.so

include $(BUILDSYSDIR)/base.mk

