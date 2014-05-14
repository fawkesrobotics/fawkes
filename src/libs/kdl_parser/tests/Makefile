#*****************************************************************************
#               Makefile Build System for Fawkes: KDL Parser Unit Test
#                            -------------------
#   Created on Mon Mar 24 15:46:30 2014
#   Copyright (C) 2014
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
include $(BASEDIR)/src/libs/kdl_parser/kdl_parser.mk

LIBS_gtest_kdlparser += stdc++ fawkeskdl_parser gtest gtest_main pthread

OBJS_gtest_kdlparser += test_kdlparser.o
OBJS_all    = $(OBJS_gtest_kdlparser)
ifeq ($(HAVE_KDLPARSER),1)
  CFLAGS += $(CFLAGS_KDLPARSER)
  LDFLAGS += $(LDFLAGS_KDLPARSER)
  BINS_test		= $(BINDIR)/gtest_kdlparser
endif


include $(BUILDSYSDIR)/base.mk
