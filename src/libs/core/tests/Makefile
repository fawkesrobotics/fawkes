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
include $(BASEDIR)/etc/buildsys/catch2.mk

LIBS_test_circular_buffer += stdc++ fawkescore m
OBJS_test_circular_buffer += test_circular_buffer.o

LIBS_test_wait_condition += stdc++ fawkescore pthread
OBJS_test_wait_condition += test_wait_condition.o

OBJS_all    = $(OBJS_test_circular_buffer) $(OBJS_test_wait_condition)

ifeq ($(HAVE_CATCH2),1)
  CFLAGS_test_circular_buffer += $(CFLAGS_CATCH2)
  LDFLAGS_test_circular_buffer += $(LDFLAGS_CATCH2)
  BINS_catch2test += $(BINDIR)/test_circular_buffer
else
  WARN_TARGETS += warning_catch2
endif

ifeq ($(HAVE_GTEST),1)
  CFLAGS_test_wait_condition += $(CFLAGS_GTEST)
  LDFLAGS_test_wait_condition += $(LDFLAGS_GTEST)
  ifeq ($(CC),clang)
    CFLAGS_test_wait_condition += -Wno-unused-variable
  else
    CFLAGS_test_wait_condition += -Wno-unused-but-set-variable
  endif
  BINS_gtest += $(BINDIR)/test_wait_condition
else
  WARN_TARGETS += warning_gtest
endif

ifeq ($(OBJSSUBMAKE),1)
test: $(WARN_TARGETS)

.PHONY: $(WARN_TARGETS)
warning_gtest:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting unit tests for WaitCondition$(TNORMAL) (gtest not available)"
warning_catch2:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting unit tests for CircularBuffer$(TNORMAL) (catch2 not available)"
endif

include $(BUILDSYSDIR)/base.mk
