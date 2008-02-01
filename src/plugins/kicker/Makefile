#*****************************************************************************
#                 Makefile Build System for Fawkes: Kicker Plugin
#                            -------------------
#   Created on Mon May 15 11:06:17 2007
#   copyright (C) 2007 by Daniel Beck, AllemaniACs RoboCup Team
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
include $(BASEDIR)/etc/buildsys/config.mk

CFLAGS = -g

ifneq ($(HAVE_IOWKIT),1)
  WARN_TARGETS += warning_iowkit
else
  PLUGINS_all = $(PLUGINDIR)/kicker.so
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_iowkit
warning_iowkit:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting Kicker Plugin$(TNORMAL) (install IowKit)"
endif

LIBS_kicker = core utils iowkit
OBJS_kicker = kicker_plugin.o	\
		     kicker_thread.o	\
                     kicker_control.o

OBJS_all    = $(OBJS_kicker)

include $(BASEDIR)/etc/buildsys/base.mk

