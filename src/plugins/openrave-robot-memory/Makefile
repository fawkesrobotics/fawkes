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
include $(BASEDIR)/src/plugins/mongodb/mongodb.mk

LIBS_openrave_robot_memory = m fawkescore fawkesutils fawkesaspects fawkesbaseapp \
                      fawkesblackboard fawkesinterface fawkesrobotmemory \
                      OpenraveRobotMemoryInterface

OBJS_openrave_robot_memory = openrave-robot-memory_plugin.o openrave-robot-memory_thread.o

OBJS_all = $(OBJS_openrave-robot-memory)

ifeq ($(HAVE_MONGODB),1)
  PLUGINS_all = $(PLUGINDIR)/openrave-robot-memory.$(SOEXT)
else
  ifneq ($(HAVE_MONGODB),1)
    WARN_TARGETS += warning_mongodb
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_mongodb
warning_mongodb:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting openrave-robot-memory Plugin$(TNORMAL) (mongodb[-devel] not installed)"
endif


include $(BUILDSYSDIR)/base.mk