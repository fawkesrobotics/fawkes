#*****************************************************************************
#              Makefile Build System for Fawkes : Kicker QA
#                            -------------------
#   Created on Mon May 14 09:45:14 2007
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

BASEDIR = ../../../..
include $(BASEDIR)/etc/buildsys/config.mk
include $(BASEDIR)/src/plugins/kicker/kicker.mk

CFLAGS = -g
INCDIRS += $(KICKER_INCDIRS)
LIBDIRS += $(KICKER_LIBDIRS)

ifneq ($(SRCDIR),.)
.PHONY: print_unsupported
print_unsupported:
	$(SILENT)if [ "$(HAVE_IOWKIT)" != "1" ]; then \
		echo -e "$(INDENT_PRINT)--> $(TRED)Omitting Kicker Plugin$(TNORMAL) (No IowKit installed)"; \
	fi \

all: print_unsupported
endif

LIBS_qa_kicker_control = stdc++ iowkit
OBJS_qa_kicker_control = qa_kicker_control.o		\
		         ../kicker_control.o

OBJS_all =	$(OBJS_qa_kicker_control)

ifeq ($(HAVE_IOWKIT),1)
  BINS_all =	$(BINDIR)/qa_kicker_control
endif

include $(BASEDIR)/etc/buildsys/base.mk

