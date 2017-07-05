#*****************************************************************************
#         Makefile Build System for Fawkes: pddl-planner Plugin
#                            -------------------
#   Created on Wed Dec  7 19:09:44 2016
 
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

PRESUBDIRS = interfaces

LIBS_pddl_planner = m fawkescore fawkesutils fawkesaspects fawkesbaseapp \
                      fawkesblackboard fawkesinterface fawkesrobotmemory \
                      PddlPlannerInterface

OBJS_pddl_planner = pddl-planner_plugin.o pddl-planner_thread.o

OBJS_all = $(OBJS_pddl_planner)

ifeq ($(HAVE_CPP11)$(HAVE_MONGODB),11)
  CFLAGS  += $(CFLAGS_CPP11)
  LDFLAGS += $(LDFLAGS_CTEMPLATE)
  PLUGINS_all = $(PLUGINDIR)/pddl-planner.$(SOEXT)
else
  ifneq ($(HAVE_CPP11),1)
    WARN_TARGETS += warning_cpp11
  endif
  ifneq ($(HAVE_TF),1)
    WARN_TARGETS = warning_mongodb
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_mongodb warning_cpp11
warning_mongodb:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting pddl-robot-memory plugin$(TNORMAL) (mongodb not available)"
warning_cpp11:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build pddl-robot-memory plugin$(TNORMAL) (C++11 not supported)"
endif

include $(BUILDSYSDIR)/base.mk
