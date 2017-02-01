#*****************************************************************************
#               Makefile Build System for Fawkes: Gazebo plugin for motor
#
#   Created on Tue Jan 28 13:30:39 2014
#   Copyright (C) 2011 by Tim Niemueller, AllemaniACs RoboCup Team
#                         Frederik Zwilling
#
##*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

BASEDIR = ../../../../..

include $(BASEDIR)/etc/buildsys/config.mk
include $(BASEDIR)/src/plugins/gazebo/gazebo.mk
include $(BUILDSYSDIR)/protobuf.mk

GAZEBO_LIBDIR = $(LIBDIR)/gazebo
LIBDIRS_BASE += $(GAZEBO_LIBDIR)

LIBS_gazebo_libmotor = gazsim_msgs
OBJS_gazebo_libmotor = motor.o

INST_LIB_SUBDIR_gazebo_libmotor = $(FFLIBSUBDIR)
NOSOVER_gazebo_libmotor=1

OBJS_all    = $(OBJS_gazebo_libmotor)

ifeq ($(HAVE_GAZEBO)$(HAVE_PROTOBUF),11)
  CFLAGS  += $(CFLAGS_GAZEBO) $(CFLAGS_PROTOBUF)
  LDFLAGS += $(LDFLAGS_GAZEBO) $(LDFLAGS_PROTOBUF) -lm $(call boost-libs-ldflags,system) -lboost_system

  LIBS_all = $(LIBDIR)/gazebo/libmotor.so
else
  ifneq ($(HAVE_GAZEBO),1)
    WARN_TARGETS += warning_gazebo
  endif
  ifneq ($(HAVE_PROTOBUF),1)
    WARN_TARGETS += warning_protobuf
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
.PHONY: warning_gazebo warning_protobuf
warning_gazebo:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting gazebo-motor Plugin$(TNORMAL) " \
		"(Gazebo Simulator not found)"
warning_protobuf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting gazebo-motor Plugin$(TNORMAL) " \
		"(protobuf[-devel] not installed)"
endif

include $(BUILDSYSDIR)/base.mk
