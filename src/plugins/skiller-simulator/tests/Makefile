#*****************************************************************************
#                      Makefile Build System for Fawkes
#                            -------------------
#   Created on Sun 22 Dec 2019 19:38:06 CET
#   Copyright (C) 2019 by Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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

LIBS_gtest_skiller_simulator_skill_parser = m fawkes_skiller_time_estimator
OBJS_gtest_skiller_simulator_skill_parser = test_skill_parser.o
OBJS_all = $(OBJS_gtest_skiller_simulator_skill_parser)
BINS_all = $(BINDIR)/gtest_skiller_simulator_skill_parser

ifeq ($(HAVE_CPP17),1)
  ifeq ($(HAVE_GTEST),1)
    CFLAGS += $(CFLAGS_GTEST)
    LDFLAGS += $(LDFLAGS_GTEST)
    BINS_test = $(BINS_all)
  else
    WARN_TARGETS += warn_gtest
  endif
else
  WARN_TARGETS += warning_cpp17
endif

ifeq ($(OBJSSUBMAKE),1)
test: $(WARN_TARGETS)
.PHONY: $(WARN_TARGETS)
warning_cpp17:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting skiller simulator tests$(TNORMAL) (C++17 not supported)"
warning_gtest:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build skiller simulator tests$(TNORMAL) (gtest not found)"
endif

include $(BUILDSYSDIR)/base.mk
