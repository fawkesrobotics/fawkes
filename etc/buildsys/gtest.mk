#*****************************************************************************
#                Makefile Build System for Fawkes: gtest unit tests
#                            -------------------
#   Created on Mon Jan 05 15:44:42 2015
#   Copyright (C) 2015-2016 by Till Hofmann
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

__GTEST_INCLUDE_PATHS=/usr/include /usr/local/include

# gtest uses its own config tool
GTESTCONFIG = $(shell which gtest-config 2>/dev/null)

ifneq ($(GTESTCONFIG),)
  HAVE_GTEST = 1
  CFLAGS_GTEST  = $(shell $(GTESTCONFIG) --cppflags --cxxflags)
  LDFLAGS_GTEST = $(shell $(GTESTCONFIG) --ldflags --libs)
else
  # gtest-config not available, but maybe we can still find gtest
  HAVE_GTEST_HDR = $(if $(wildcard $(addsuffix /gtest/gtest.h,$(__GTEST_INCLUDE_PATHS))),1)
  ifeq ($(HAVE_GTEST_HDR),1)
    # check if library is available
    HAVE_GTEST_LIB = $(if $(shell ldconfig -p | grep libgtest\\.$(SOEXT)),1,)
    HAVE_GTEST_MAIN = $(if $(shell ldconfig -p | grep libgtest_main\\.$(SOEXT)),1,)
    ifeq ($(HAVE_GTEST_LIB)$(HAVE_GTEST_MAIN),11)
      HAVE_GTEST = 1
      # gtest-config not available, we must set the flags manually
      LDFLAGS_GTEST += -pthread -lgtest
    endif
  endif
endif

ifeq ($(HAVE_GTEST),1)
  # always link against gtest_main
  # this defines main() for all gtests
  LDFLAGS_GTEST += -lgtest_main

  # gtest binaries don't need a man page
  WARN_MISSING_MANPAGE = 0
endif

endif # __buildsys_gtest_mk_
