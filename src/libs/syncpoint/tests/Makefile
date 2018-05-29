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

CFLAGS += -Wno-unused-but-set-variable -Wno-unused-variable
LIBS_gtest_syncpoint += stdc++ fawkescore fawkesutils fawkessyncpoint pthread \
                        fawkeslogging

OBJS_gtest_syncpoint += test_syncpoint.o
OBJS_all = $(OBJS_gtest_syncpoint)

ifeq ($(HAVE_GTEST)$(HAVE_CPP11),11)
  CFLAGS += $(CFLAGS_GTEST) $(CFLAGS_CPP11)
  LDFLAGS += $(LDFLAGS_GTEST)
  LIBS_test = $(LIBDIR)/test/syncpoint.so
  BINS_test = $(BINDIR)/gtest_syncpoint
endif

include $(BUILDSYSDIR)/base.mk
