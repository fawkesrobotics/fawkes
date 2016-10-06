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
include $(BUILDCONFDIR)/tf/tf.mk
include $(BASEDIR)/src/plugins/mongodb/mongodb.mk
include $(BASEDIR)/etc/buildsys/boost.mk

PRESUBDIRS = interfaces aspect
SUBDIRS = test-plugin

REQ_BOOST_LIBS = bind function
HAVE_BOOST_LIBS = $(call boost-have-libs,$(REQ_BOOST_LIBS))

LIBS_robot_memory = fawkescore fawkesaspects fawkesblackboard fawkesinterface \
		fawkesutils fawkeslogging fawkesmongodbaspect fvutils \
		fawkestf RobotMemoryInterface fawkesrobotmemory
OBJS_robot_memory = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))

OBJS_all    = $(OBJS_robot_memory)

ifeq ($(HAVE_MONGODB)$(HAVE_TF)$(HAVE_BOOST_LIBS),111)
  CFLAGS += $(CFLAGS_TF) $(CFLAGS_MONGODB) $(call boost-libs-cflags,$(REQ_BOOST_LIBS))
  LDFLAGS += $(LDFLAGS_TF) $(LDFLAGS_MONGODB) $(call boost-libs-ldflags,$(REQ_BOOST_LIBS))
  PLUGINS_all = $(PLUGINDIR)/robot-memory.so
else
  ifneq ($(HAVE_MONGODB),1)
    WARN_TARGETS += warning_mongodb
  endif
  ifneq ($(HAVE_TF),1)
    WARN_TARGETS += warning_tf
  endif
  ifneq ($(HAVE_BOOST_LIBS),1)
    WARN_TARGETS_BOOST = $(foreach l,$(REQ_BOOST_LIBS),$(if $(call boost-have-lib,$l),, warning_boost_$l))
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_mongodb warning_tf
warning_mongodb:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting RobotMemory Plugin$(TNORMAL) (mongodb[-devel] not installed)"
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting RobotMemory Plugin$(TNORMAL) (tf framework not available)"
$(WARN_TARGETS_BOOST): warning_boost_%:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build RobotMemory Plugin$(TNORMAL) (Boost library $* not found)"endif
endif

include $(BUILDSYSDIR)/base.mk

