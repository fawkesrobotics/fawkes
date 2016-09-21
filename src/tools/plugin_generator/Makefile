#*****************************************************************************
#            Makefile Build System for Fawkes : Plugin Generator
#                            -------------------
#   Created on Tue Apr 05 2016
#   copyright (C) 2016 by Johannes Rothe, Carologistics RoboCup Team
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
include $(BUILDSYSDIR)/interface.mk

ifeq ($(HAVE_CPP11),1)
	CFLAGS += $(CFLAGS_CPP11)
else
	WARN_TARGETS += warning_cpp11
endif

LIBS_ffgenplugin = stdc++ fawkescore fawkesutils

OBJS_ffgenplugin = plugin_generator.o main.o

OBJS_all     += $(OBJS_ffgenplugin)
BINS_all     += $(BINDIR)/ffgenplugin
MANPAGES_all = $(MANDIR)/man1/ffgenplugin.1

ifeq ($(OBJSSUBMAKE),1)
  ifneq ($(WARN_TARGETS),)
all: $(WARN_TARGETS)
  endif
.PHONY: warning_cpp11
warning_cpp11:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting plugin generator$(TNORMAL) (C++11 not supported)"
endif

include $(BUILDSYSDIR)/base.mk
