#*****************************************************************************
#                Makefile Build System for Fawkes: gtest unit tests
#                            -------------------
#   Created on Mon Jan 05 15:44:42 2015
#   Copyright (C) 2015 by Till Hofmann
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

ifndef __buildsys_config_mk_
$(error config.mk must be included before gtest.mk)
endif

ifndef __buildsys_gtest_mk_
__buildsys_gtest_mk_ := 1

# gtest uses its own config tool
GTESTCONFIG = $(shell which gtest-config)

ifneq ($(GTESTCONFIG),)
  HAVE_GTEST = 1
endif

ifeq ($(HAVE_GTEST),1)
  CFLAGS_GTEST  = $(shell $(GTESTCONFIG) --cppflags --cxxflags)
  LDFLAGS_GTEST = $(shell $(GTESTCONFIG) --ldflags --libs)

  # always link against gtest_main
  # this defines main() for all gtests
  LDFLAGS_GTEST += -lgtest_main

  # gtest binaries don't need a man page
  WARN_MISSING_MANPAGE = 0
endif

endif # __buildsys_gtest_mk_
