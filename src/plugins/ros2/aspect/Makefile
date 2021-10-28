#*****************************************************************************
#                Makefile Build System for Fawkes: ROS Aspect
#                            -------------------
#   Created on Thu May 05 15:52:04 2011
#   Copyright (C) 2006-2011 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(LIBSRCDIR)/utils/utils.mk
include $(BUILDSYSDIR)/ros2.mk

CFLAGS  += $(CFLAGS_ROS2)
LDFLAGS += $(LDFLAGS_ROS2)

LIBS_libfawkesros2aspect = stdc++ fawkescore fawkesaspects fawkesutils
OBJS_libfawkesros2aspect = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))

OBJS_all = $(OBJS_libfawkesros2aspect)
LIBS_all = $(LIBDIR)/libfawkesros2aspect.so

ifeq ($(HAVE_ROS2),1)
  LIBS_build = $(LIBS_all)
else
  WARN_TARGETS += warning_ros2
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_ros2
warning_ros2:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS2 Aspect$(TNORMAL) (ROS2 not found)"
endif

include $(BUILDSYSDIR)/base.mk

