#*****************************************************************************
#           Makefile Build System for Fawkes : FireVision Fountain
#                            -------------------
#   Created on Sun Jun 17 17:47:54 2007 (Starbucks)
#   Copyright (C) 2006-2007 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BUILDSYSDIR)/fvconf.mk

CFLAGS   += $(VISION_CFLAGS)
LDFLAGS  += $(VISION_LDFLAGS)
INCDIRS  += $(VISION_INCDIRS)
LIBDIRS  += $(VISION_LIBDIRS)
LIBS     += $(VISION_LIBS)

LIBS_fvfountain = fawkescore fawkesutils fvutils fawkesnetcomm fawkesaspects
OBJS_fvfountain  = fountain_thread.o fountain_plugin.o
PLUGINS_all      = $(PLUGINDIR)/fvfountain.so

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
.PHONY: warning_avahi
warning_avahi:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Fountain will be built without mDNS-SD$(TNORMAL) (avahi-devel not installed)"
endif

OBJS_all = $(OBJS_fvfountain)

include $(BUILDSYSDIR)/base.mk

