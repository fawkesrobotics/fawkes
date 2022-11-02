#*****************************************************************************
#                      Makefile Build System for Fawkes
#                            -------------------
#   Created on Thu 8 Mar 2022 16:53:02 CEST
#   Copyright (C)
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
include $(LIBSRCDIR)/utils/utils.mk
include $(BUILDSYSDIR)/pddl_parser.mk
include $(FAWKES_BASEDIR)/src/plugins/mongodb/mongodb.mk
#include $(BASEDIR)/src/plugins/robot-memory/robot_memory.mk
#include $(BUILDSYSDIR)/protobuf.mk
include $(BUILDSYSDIR)/boost.mk
include $(BUILDSYSDIR)/clips.mk
#include $(BUILDCONFDIR)/tf/tf.mk
BOOST_ROOT = "/usr/local/boost_1_76_0"


LIBS_rl_test = fawkescore fawkesutils fawkesaspects fawkesbaseapp \
               fawkesblackboard fawkesinterface fawkesclipsaspect\
               fawkeslogging fawkesmongodbaspect fvutils \
               python3.10 \
               RLAgentGoalSelectionInterface
#boost_python310 \

IDIR+=/usr/include/python3.10/ #/home/sonja/.local/lib/python3.9/
CFLAGS+=$(python3 -m pybind11 --includes) -c -I$(IDIR) $(pyton3-config --extension-suffix) -Wno-error #-c -I$(IDIR)

LD_LIBRARY_PATH=$(LD_LIBRARY_PATH):~/.local/lib/python3.10/
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/Documents/rlblocksworld/src-python/
#vpath %.py /home/sonja/.local/lib/python3.9/

# LDIR=/usr/lib64
#LIBS=boost_python39 python3.9
# #usr/lib64/libboost_python39.so


#SOURCES=rl-test-plugin.cpp rl-test-thread.cpp
#OBJECTS=$(SOURCES:.cpp=.o)
#EXECUTABLE=rl-test

LDFLAGS+=-L.~/Documents/rlblocksworld/src-cpp/

OBJS_rl_test = $(patsubst %.cpp,%.o,$(subst $(SRCDIR) -DBOOST_PYTHON_STATIC_LIB/,,$(wildcard $(SRCDIR)/*.cpp)))
#OBJS_rl_test = rl-test-plugin.o rl-test-thread.o #$@ -I$(IDIR) $(LIBS)

OBJS_all = $(OBJS_rl_test)
PLUGINS_all = $(PLUGINDIR)/rl-test.so


ifeq ($(HAVE_CPP17),1)
  CFLAGS        += $(CFLAGS_CPP17)
  PLUGINS_build  = $(PLUGINS_all)
else
  WARN_TARGETS += warning_cpp17
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
.PHONY: $(WARN_TARGETS)

warning_cpp17:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting rl test plugin$(TNORMAL) (C++17 not supported)"
endif

ifeq ($(HAVE_CLIPS),1)
  CFLAGS  += $(CFLAGS_CLIPS)
  LDFLAGS += $(LDFLAGS_CLIPS)

  PLUGINS_build = $(PLUGINS_all)
else
  WARN_TARGETS += warning_clips
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_clips
warning_clips:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting CEDAR plugin$(TNORMAL) (CLIPS not found)"
endif

include $(BUILDSYSDIR)/base.mk
