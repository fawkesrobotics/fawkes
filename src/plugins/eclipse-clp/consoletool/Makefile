#*****************************************************************************
#        Makefile Build System for Fawkes : Skiller Console Tool
#                            -------------------
#   Created on Sat Mar 15 13:54:37 2008
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
include $(BASEDIR)/fawkes/src/plugins/skiller/skiller.mk

LIBS_skilltester = stdc++ fawkescore fawkesutils fawkesnetcomm fawkesblackboard \
		fawkesinterface \
		readline history $(if $(HAVE_TERMCAP),termcap) \
		TestInterface
OBJS_skilltester = skilltester.o

OBJS_all =	$(OBJS_skillet)

ifeq ($(HAVE_READLINE),1)
  BINS_all =	$(BINDIR)/skilltester
  MANPAGES_all = $(MANDIR)/man1/skilltester.1
else
  WARN_TARGETS += warning_readline
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_readline
warning_readline:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting Skilltester console tool$(TNORMAL) (readline[-devel] not installed)"
endif


include $(BUILDSYSDIR)/base.mk

