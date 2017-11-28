#*****************************************************************************
#                      Makefile Build System for Fawkes
#                            -------------------
#   Created on Tue 19 Sep 2017 13:41:20 CEST
#   Copyright (C) 2017 by Till Hofmann <hofmann@kbsg.rwth-aachen.de>
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

BASEDIR = ../../../../..
include $(BASEDIR)/etc/buildsys/config.mk
include $(BUILDSYSDIR)/clips.mk
include $(BUILDSYSDIR)/gtest.mk

OBJS_gtest_clips_example += test_example.o
OBJS_gtest_clips_domain += test_domain.o
OBJS_all = $(OBJS_gtest_clips_example)

ifeq ($(HAVE_CLIPS),1)
  ifeq ($(HAVE_GTEST),1)
    CFLAGS += $(CFLAGS_CLIPS) $(CFLAGS_GTEST) -DTESTDIR=\"$(PWD)\"
    LDFLAGS += $(LDFLAGS_CLIPS) $(LDFLAGS_GTEST)
    BINS_test += $(BINDIR)/gtest_clips_example
    BINS_test += $(BINDIR)/gtest_clips_domain
  else
    WARN_TARGETS += warning_gtest
  endif
else
  WARN_TARGETS += warning_clips
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_gtest warning_clips
warning_gtest:
	$(SILENT)echo -e "$(INDENT_PRINT) --> $(TRED)Omitting CLIPS Tests$(TNORMAL) (gtest not available)"
warning_clips:
	$(SILENT)echo -e "$(INDENT_PRINT) --> $(TRED)Omitting CLIPS Tests$(TNORMAL) (CLIPS not available)"
endif

include $(BUILDSYSDIR)/base.mk
