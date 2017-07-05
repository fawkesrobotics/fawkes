#*****************************************************************************
#        Makefile Build System for Fawkes: Unit testing in Fawkes
#                            -------------------
#   Created on Aug 24, 2016 11:40:46 PM 2016
#   Copyright (C) 2016 by Frederik Zwilling
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
include $(BASEDIR)/etc/buildsys/gtest.mk
include $(BASEDIR)/etc/buildsys/config.mk
include $(BASEDIR)/src/libs/netcomm/netcomm.mk

LIBS_gtest_fawkes += stdc++ fawkescore fawkesutils fawkesblackboard fawkesnetcomm \
	fawkesconfig fawkesplugin fawkesaspects fawkesbaseapp
OBJS_gtest_fawkes += gtest_fawkes.o

OBJS_all    = $(OBJS_gtest_fawkes)

ifeq ($(HAVE_GTEST)$(HAVE_CPP11),11)
  CFLAGS += $(CFLAGS_GTEST)
  CFLAGS += -Wno-unused-but-set-variable
  CFLAGS += $(CFLAGS_CPP11)
  LDFLAGS += $(LDFLAGS_GTEST)
  LIBS_test = $(LIBDIR)/test/fawkes.so
  BINS_test = $(BINDIR)/gtest_fawkes
else
  WARN_TARGETS += warning_gtest
endif

ifeq ($(OBJSSUBMAKE),1)
test: $(WARN_TARGETS)

.PHONY: warning_gtest
warning_gtest:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting unit tests for Fawkes$(TNORMAL) (gtest or cpp11 not available)"

endif

include $(BUILDSYSDIR)/base.mk
