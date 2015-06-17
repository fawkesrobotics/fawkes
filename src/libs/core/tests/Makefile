#*****************************************************************************
#        Makefile Build System for Fawkes: CircularBuffer Unit Test
#                            -------------------
#   Created on Fri Aug 15 16:33:42 2014
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

LIBS_gtest_circular_buffer += stdc++ fawkescore fawkesutils
OBJS_gtest_circular_buffer += test_circular_buffer.o

LIBS_gtest_wait_condition += stdc++ fawkescore pthread
OBJS_gtest_wait_condition += test_wait_condition.o

OBJS_all    = $(OBJS_gtest_circular_buffer) $(OBJS_gtest_wait_condition)

ifeq ($(HAVE_GTEST),1)
  CFLAGS += $(CFLAGS_GTEST)
  CFLAGS += -Wno-unused-but-set-variable
  LDFLAGS += $(LDFLAGS_GTEST)
  LIBS_test = $(LIBDIR)/test/circular_buffer.so
  BINS_test = $(BINDIR)/gtest_circular_buffer
  LIBS_test += $(LIBDIR)/test/wait_condition.so
  BINS_test += $(BINDIR)/gtest_wait_condition
else
  WARN_TARGETS += warning_gtest
endif

ifeq ($(OBJSSUBMAKE),1)
test: $(WARN_TARGETS)

.PHONY: warning_gtest
warning_gtest:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting unit tests for CircularBuffer and WaitCondition$(TNORMAL) (gtest not available)"

endif

include $(BUILDSYSDIR)/base.mk
