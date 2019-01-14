#*****************************************************************************
#        Makefile Build System for Fawkes: Google Test Library
#                            -------------------
#   Created on Mon Jan 14 09:59:14 2019 +0100
#   Copyright (C) 2006-2018 by Tim Niemueller [www.niemueller.org]
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

LIBS_libfawkesgtest = pthread
SRCS_libfawkesgtest = src/gtest-death-test.cpp src/gtest-filepath.cpp src/gtest-port.cpp \
  src/gtest-printers.cpp src/gtest-test-part.cpp src/gtest-typed-test.cpp \
  src/gtest.cpp
OBJS_libfawkesgtest = $(patsubst %.cpp,%.o,$(SRCS_libfawkesgtest))

LIBS_libfawkesgtest_main = fawkesgtest
SRCS_libfawkesgtest_main = src/gtest_main.cpp
OBJS_libfawkesgtest_main = $(patsubst %.cpp,%.o,$(SRCS_libfawkesgtest_main))

CLEAN_FILES=src gtest

OBJS_all = $(OBJS_libfawkesgtest) $(OBJS_libfawkesgtest_main)

$(SRCS_libfawkesgtest) $(SRCS_libfawkesgtest_main): %.cpp: $(SRCDIR)/src/gtest-internal-inl.h

$(SRCDIR)/src/%.cpp: /usr/src/googletest/googletest/src/%.cc
	$(eval BUILT_PARTS += $@)
	$(SILENTSYMB) echo -e "$(INDENT_PRINT)[CPY] $< -> $(TBOLDGRAY)$(subst $(SRCDIR)/,,$@)$(TNORMAL)"
	$(SILENT)mkdir -p $(@D)
	$(SILENT)cp -af $< $@

$(SRCDIR)/src/%.h: /usr/src/googletest/googletest/src/%.h
	$(eval BUILT_PARTS += $@)
	$(SILENTSYMB) echo -e "$(INDENT_PRINT)[CPY] $< -> $(TBOLDGRAY)$(subst $(SRCDIR)/,,$@)$(TNORMAL)"
	$(SILENT)mkdir -p $(@D)
	$(SILENT)cp -af $< $@

$(OBJS_all): | $(SRCDIR)/gtest $(addprefix $(SRCDIR)/,$(SRCS_libfawkesgtest) $(SRCS_libfawkesgtest_main))

$(SRCDIR)/gtest: /usr/src/googletest/googletest/include/gtest
	$(eval BUILT_PARTS += $@)
	$(SILENTSYMB) echo -e "$(INDENT_PRINT)[CPY] $< -> $(TBOLDGRAY)$(PARENTDIR)$(subst $(SRCDIR)/,,$@)$(TNORMAL)"
	$(SILENT)cp -af $< $@

$(LIBDIR)/libfawkesgtest_main.$(SOEXT): | $(LIBDIR)/libfawkesgtest.$(SOEXT)

ifeq ($(DISTRO),ubuntu)
  LIBS_all  = $(LIBDIR)/libfawkesgtest.$(SOEXT) $(LIBDIR)/libfawkesgtest_main.$(SOEXT)
else
  INFO_TARGETS += info_sysgtest
endif

ifeq ($(OBJSSUBMAKE),1)
  ifneq ($(WARN_TARGETS)$(INFO_TARGETS),)
all: $(WARN_TARGETS) $(INFO_TARGETS)
  endif
.PHONY: info_sysgtest
info_sysgtest:
	$(SILENT)echo -e "$(INDENT_PRINT)--- Using system gtest library"
endif

include $(BUILDSYSDIR)/base.mk

