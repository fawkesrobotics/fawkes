#*****************************************************************************
#         Makefile Build System for Fawkes : Fawkes Process Watchdog
#                            -------------------
#   Created on Thu Mar 31 09:52:29 2011
#   Copyright (C) 2006-2011 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

BASEDIR = ../../..

include $(BASEDIR)/etc/buildsys/config.mk

LIBS_ffwatchdog = stdc++ fawkescore
OBJS_ffwatchdog = ffwatchdog.o

OBJS_all     =	$(OBJS_ffwatchdog)
BINS_all     =	$(BINDIR)/ffwatchdog
MANPAGES_all =	$(MANDIR)/man1/ffwatchdog.1

ifeq ($(OBJSSUBMAKE),1)
  ifneq ($(WARN_TARGETS),)
all: $(WARN_TARGETS)
  endif
.PHONY: warning_libdaemon
warning_libdaemon:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)Building without daemonizing support$(TNORMAL) (libdaemon[-devel] not installed)"
endif


include $(BUILDSYSDIR)/base.mk
