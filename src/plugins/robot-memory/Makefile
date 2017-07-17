#*****************************************************************************
#           Makefile Build System for Fawkes: Robot-Memory Plugin
#                            -------------------
#   Created on Sun May 01 13:32:12 2016
#   Copyright (C) 2016 Frederik Zwilling
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

PRESUBDIRS = interfaces aspect
SUBDIRS = test-plugin

LIBS_robot_memory = fawkescore fawkesaspects fawkesblackboard fawkesinterface \
		fawkesutils fawkeslogging fawkesmongodbaspect fvutils \
		fawkestf RobotMemoryInterface fawkesrobotmemory
OBJS_robot_memory = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))
OBJS_robot_memory += $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/computables/*.cpp)))))

OBJS_all    = $(OBJS_robot_memory)

ifeq ($(HAVE_ROBOT_MEMORY)$(HAVE_CPP11),11)
  CFLAGS += $(CFLAGS_ROBOT_MEMORY) $(CFLAGS_CPP11)
  LDFLAGS += $(LDFLAGS_ROBOT_MEMORY)
  PLUGINS_all = $(PLUGINDIR)/robot-memory.so
else
  ifneq ($(HAVE_MONGODB),1)
    WARN_TARGETS += warning_mongodb
  endif
  ifneq ($(HAVE_CPP11),1)
    WARN_TARGETS += warning_cpp11
  endif
  ifneq ($(HAVE_TF),1)
    WARN_TARGETS += warning_tf
  endif
  ifeq ($(OLDER_THAN_XENIAL),1)
    WARN_TARGETS += warning_ubuntu
  endif
  ifneq ($(HAVE_BOOST_LIBS),1)
    WARN_TARGETS_BOOST = $(foreach l,$(REQ_BOOST_LIBS),$(if $(call boost-have-lib,$l),, warning_boost_$l))
  endif
endif


ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_cpp11 warning_mongodb warning_tf warning_ubuntu
warning_cpp11:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting RobotMemory Plugin$(TNORMAL) (C++11 required)"
warning_mongodb:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting RobotMemory Plugin$(TNORMAL) (mongodb[-devel] not installed)"
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting RobotMemory Plugin$(TNORMAL) (tf framework not available)"
warning_ubuntu:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting RobotMemory Plugin$(TNORMAL) (Ubuntu version is too old)"
$(WARN_TARGETS_BOOST): warning_boost_%:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build RobotMemory Plugin$(TNORMAL) (Boost library $* not found)"endif
endif

include $(BUILDSYSDIR)/base.mk

