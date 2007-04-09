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

LIBS_example_plugin = core utils
OBJS_example_plugin = example_plugin.o	\
		      thread.o

OBJS_all    = $(OBJS_example_plugin)
PLUGINS_all = $(PLUGINDIR)/example_plugin.so

include $(BASEDIR)/etc/buildsys/base.mk
