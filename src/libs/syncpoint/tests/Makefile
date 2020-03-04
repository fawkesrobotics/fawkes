#*****************************************************************************
#               Makefile Build System for Fawkes: SyncPoint Unit Test
#                            -------------------
#   Created on Wed Jan 22 16:07:03 2014
#   Copyright (C) 2014 by Till Hofmann
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
include $(BASEDIR)/etc/buildsys/gtest.mk

LIBS_test_syncpoint += stdc++ fawkescore fawkesutils fawkessyncpoint pthread \
                        fawkeslogging

OBJS_test_syncpoint += test_syncpoint.o
OBJS_all = $(OBJS_test_syncpoint)

CFLAGS += -Wno-unused-variable
ifneq ($(CC),clang)
  CFLAGS += -Wno-unused-but-set-variable
endif

ifeq ($(HAVE_GTEST)$(HAVE_CPP11),11)
  CFLAGS += $(CFLAGS_GTEST) $(CFLAGS_CPP11)
  LDFLAGS += $(LDFLAGS_GTEST)
  BINS_gtest = $(BINDIR)/test_syncpoint
else
  ifneq ($(HAVE_GTEST),1)
    WARN_TARGETS += warning_gtest
  endif
  ifneq ($(HAVE_CPP11),1)
    WARN_TARGETS += warning_cpp11
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
test: $(WARN_TARGETS)
.PHONY: $(WARN_TARGETS)
warning_gtest:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build syncpoint tests$(TNORMAL) (gtest not found)"
warning_cpp11:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build syncpoint tests$(TNORMAL) (C++11 not supported)"
endif

include $(BUILDSYSDIR)/base.mk
