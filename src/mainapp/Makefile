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
#
#           $Id$
# last modified: $Date$
#            by: $Author$
#

BASEDIR = ../..

CFLAGS = -g -DLIBDIR=\"$(LIBDIR)\" -DPLUGINDIR=\"$(PLUGINDIR)\"

LIBS_fawkes = core utils interfaces blackboard netcomm
OBJS_fawkes = \
	      main.o					\
	      main_thread.o				\
	      network_manager.o				\
	      plugin_manager.o				\
	      thread_initializer.o

OBJS_all =	$(OBJS_fawkes)
BINS_all =	$(BINDIR)/fawkes

include $(BASEDIR)/etc/buildsys/base.mk

