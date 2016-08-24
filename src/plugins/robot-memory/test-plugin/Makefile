#*****************************************************************************
#         Makefile Build System for Fawkes: robot_memory_test Plugin
#                            -------------------
#   Created on Wed Aug 24 13:37:27 2016
 
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

BASEDIR = ../../../..
include $(BASEDIR)/etc/buildsys/config.mk
include $(BASEDIR)/etc/buildsys/gtest.mk

LIBS_robot_memory_test = m fawkescore fawkesutils fawkesaspects fawkesbaseapp \
                      fawkesblackboard fawkesinterface fawkesrobotmemory

OBJS_robot_memory_test = robot_memory_test_plugin.o robot_memory_test_thread.o robot_memory_test.o

OBJS_all = $(OBJS_robot_memory_test)

ifeq ($(HAVE_GTEST)$(HAVE_CPP11),11)
  CFLAGS += $(CFLAGS_GTEST)
  CFLAGS += $(CFLAGS_CPP11)
  LDFLAGS += $(LDFLAGS_GTEST)
  PLUGINS_all = $(PLUGINDIR)/robot_memory_test.$(SOEXT)
else
  WARN_TARGETS += warning_gtest
endif

ifeq ($(OBJSSUBMAKE),1)
test: $(WARN_TARGETS)

.PHONY: warning_gtest
warning_gtest:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting unit tests for Robot Memory$(TNORMAL) (gtest or cpp11 not available)"

endif

include $(BUILDSYSDIR)/base.mk