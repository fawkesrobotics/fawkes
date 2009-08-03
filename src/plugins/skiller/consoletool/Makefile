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
include $(BASEDIR)/src/plugins/skiller/skiller.mk

LIBS_skillet = fawkescore fawkesutils fawkesnetcomm fawkesblackboard \
		readline history $(if $(HAVE_TERMCAP),termcap) \
		SkillerInterface
OBJS_skillet = skillet.o

OBJS_all =	$(OBJS_skillet)

ifeq ($(HAVE_READLINE),1)
  BINS_all =	$(BINDIR)/skillet
else
  WARN_TARGETS += warning_readline
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_readline
warning_readline:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting Skiller console tool$(TNORMAL) (readline[-devel] not installed)"
endif


include $(BASEDIR)/etc/buildsys/base.mk

