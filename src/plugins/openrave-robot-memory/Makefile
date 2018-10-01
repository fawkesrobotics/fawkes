#*****************************************************************************
#         Makefile Build System for Fawkes: openrave-robot-memory Plugin
#                            -------------------
#   Created on Thu Nov 24 13:14:33 2016
 
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
include $(BASEDIR)/src/plugins/robot-memory/robot_memory.mk

LIBS_openrave_robot_memory = m fawkescore fawkesutils fawkesaspects fawkesbaseapp \
                      fawkesblackboard fawkesinterface fawkesrobotmemory \
                      OpenraveRobotMemoryInterface

OBJS_openrave_robot_memory = openrave-robot-memory_plugin.o openrave-robot-memory_thread.o

OBJS_all = $(OBJS_openrave_robot_memory)

ifeq ($(HAVE_CPP11)$(HAVE_ROBOT_MEMORY),11)
  CFLAGS  += $(CFLAGS_CPP11)
  PLUGINS_all = $(PLUGINDIR)/openrave-robot-memory.$(SOEXT)
else
  ifneq ($(HAVE_ROBOT_MEMORY),1)
    WARN_TARGETS += warning_robot_memory
  endif
  ifneq ($(HAVE_CPP11),1)
    WARN_TARGETS = warning_cpp11
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_robot_memory warning_cpp11
warning_robot_memory:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting openrave-robot-memory Plugin$(TNORMAL) (robot-memory not available)"
warning_cpp11:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build clips-robot-memory plugin$(TNORMAL) (C++11 not supported)"
endif


include $(BUILDSYSDIR)/base.mk
