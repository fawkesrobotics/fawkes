#*****************************************************************************
#         Makefile Build System for Fawkes: pddl_robot_memory Plugin
#                            -------------------
#   Created on Thu Oct 13 13:34:05 2016

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
include $(BASEDIR)/src/plugins/pddl-robot-memory/ctemplate.mk

LIBS_pddl_robot_memory = m fawkescore fawkesutils fawkesaspects fawkesbaseapp \
                         fawkesblackboard fawkesinterface fawkesrobotmemory \
                         PddlGenInterface

OBJS_pddl_robot_memory = pddl_robot_memory_plugin.o pddl_robot_memory_thread.o

OBJS_all   = $(OBJS_pddl_robot_memory)
PLUGINS_all = $(PLUGINDIR)/pddl-robot-memory.$(SOEXT)

ifeq ($(HAVE_CPP17)$(HAVE_ROBOT_MEMORY)$(HAVE_CTEMPLATE),111)
  PRESUBDIRS = interfaces

  CFLAGS  += $(CFLAGS_CPP17) $(CFLAGS_ROBOT_MEMORY)
  LDFLAGS += $(LDFLAGS_CTEMPLATE) $(LDFLAGS_ROBOT_MEMORY)

  PLUGINS_build = $(PLUGINS_all)
else
  ifneq ($(HAVE_CPP17),1)
    WARN_TARGETS += warning_cpp17
  endif
  ifneq ($(HAVE_ROBOT_MEMORY),1)
    WARN_TARGETS += warning_robot_memory
  endif
  ifneq ($(HAVE_CTEMPLATE),1)
    WARN_TARGETS += warning_ctemplate
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_robot_memory warning_ctemplate warning_cpp11
warning_robot_memory:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting pddl-robot-memory plugin$(TNORMAL) (robot memory not available)"
warning_cpp11:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build pddl-robot-memory plugin$(TNORMAL) (C++17 not supported)"
warning_ctemplate:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build pddl-robot-memory plugin$(TNORMAL) (ctemplate-devel not installed)"
endif

include $(BUILDSYSDIR)/base.mk
