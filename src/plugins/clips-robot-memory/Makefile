#*****************************************************************************
#         Makefile Build System for Fawkes: clips_robot_memory Plugin
#                            -------------------
#   Created on Mon Aug 29 15:41:47 2016
 
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
include $(BUILDSYSDIR)/clips.mk
include $(BASEDIR)/src/plugins/robot-memory/robot_memory.mk

LIBS_clips_robot_memory = m fawkescore fawkesutils fawkesaspects fawkesbaseapp \
                      fawkesblackboard fawkesinterface fawkesclipsaspect \
                      fawkesrobotmemory

OBJS_clips_robot_memory = clips_robot_memory_plugin.o clips_robot_memory_thread.o clips_rm_trigger.o

OBJS_all = $(OBJS_clips_robot_memory)

ifeq ($(HAVE_CPP11)$(HAVE_CLIPS)$(HAVE_ROBOT_MEMORY),111)
  CFLAGS  += $(CFLAGS_CLIPS) $(CFLAGS_CPP11)
  LDFLAGS += $(LDFLAGS_CLIPS)
  
  PLUGINS_all = $(PLUGINDIR)/clips_robot_memory.$(SOEXT)
else
  ifneq ($(HAVE_CPP11),1)
    WARN_TARGETS += warning_cpp11
  endif
  ifneq ($(HAVE_CLIPS),1)
    WARN_TARGETS = warning_clips
  endif
  ifneq ($(HAVE_ROBOT_MEMORY),1)
    WARN_TARGETS = warning_robot_memory
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_robot_memory warning_clips warning_cpp11
warning_clips:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting clips-robot-memory plugin$(TNORMAL) ($(CLIPS_ERROR))"
warning_robot_memory:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting clips-robot-memory plugin$(TNORMAL) (robot memory not available)"
warning_cpp11:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build clips-robot-memory plugin$(TNORMAL) (C++11 not supported)"

endif

include $(BUILDSYSDIR)/base.mk
