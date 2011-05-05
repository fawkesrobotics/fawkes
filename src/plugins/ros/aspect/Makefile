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
include $(BUILDSYSDIR)/ros.mk

CFLAGS  += $(CFLAGS_ROS)
LDFLAGS += $(LDFLAGS_ROS)

LIBS_libfawkesrosaspect = stdc++ fawkescore fawkesaspects fawkesutils
OBJS_libfawkesrosaspect = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))

OBJS_all    = $(OBJS_libfawkesrosaspect)

ifeq ($(HAVE_ROS),1)
  LIBS_all = $(LIBDIR)/libfawkesrosaspect.so
else
  WARN_TARGETS += warning_ros
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_ros
warning_ros:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS Aspect$(TNORMAL) (ROS not found)"
endif

include $(BUILDSYSDIR)/base.mk

