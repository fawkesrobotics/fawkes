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
include $(BASEDIR)/src/plugins/kicker/kickerconf.mk

CFLAGS = -g
INCDIRS += $(KICKER_INCDIRS)
LIBDIRS += $(KICKER_LIBDIRS)

ifneq ($(SRCDIR),)
.PHONY: print_unsupported
print_unsupported:
	$(SILENT)if [ "$(HAVE_IOWKIT)" != "1" ]; then \
		echo -e "$(INDENT_PRINT)--> $(TRED)Omitting Kicker Plugin$(TNORMAL) (No IowKit installed)"; \
	fi \

all: print_unsupported
endif

LIBS_kicker_plugin = core utils iowkit
OBJS_kicker_plugin = kicker_plugin.o	\
		     kicker_thread.o	\
                     kicker_control.o

OBJS_all    = $(OBJS_kicker_plugin)
ifeq ($(HAVE_IOWKIT),1)
  PLUGINS_all = $(PLUGINDIR)/kicker_plugin.so
endif

include $(BASEDIR)/etc/buildsys/base.mk
