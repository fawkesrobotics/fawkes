#*****************************************************************************
#                      Makefile Build System for Fawkes
#                            -------------------
#   Created on Tue 17 Nov 2020 15:06:31 CET
#   Copyright (C) 2020 by Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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
include $(BASEDIR)/etc/buildsys/catch2.mk

LIBS_test_uuid += stdc++ fawkesutils fawkescore m
OBJS_test_uuid += test_uuid.o catch2_main.o

OBJS_all = $(OBJS_test_uuid)

ifeq ($(HAVE_CATCH2),1)
  CFLAGS_test_uuid += $(CFLAGS_CATCH2)
  LDFLAGS_test_uuid += $(LDFLAGS_CATCH2)
  BINS_catch2test += $(BINDIR)/test_uuid
else
  WARN_TARGETS += warning_catch2
endif

ifeq ($(OBJSSUBMAKE),1)
test: $(WARN_TARGETS)

.PHONY: $(WARN_TARGETS)
warning_catch2:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting unit tests for CircularBuffer$(TNORMAL) (catch2 not available)"
endif

include $(BUILDSYSDIR)/base.mk
