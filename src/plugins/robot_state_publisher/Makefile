#*****************************************************************************
#      Makefile Build System for Fawkes: Robot State Publisher Plugin
#
#   Created on Thu Aug 22 11:14:00 2013
#   Copyright (C) 2013 by Till Hofmann, AllemaniACs RoboCup Team
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
include $(BUILDSYSDIR)/boost.mk
include $(BUILDSYSDIR)/eigen3.mk
include $(BUILDCONFDIR)/tf/tf.mk
include $(BUILDCONFDIR)/kdl_parser/kdl_parser.mk

LIBS_robot_state_publisher = fawkescore fawkesutils fawkesaspects fawkestf \
                             fawkesblackboard fawkesinterface JointInterface fawkeskdl_parser
OBJS_robot_state_publisher = robot_state_publisher_plugin.o robot_state_publisher_thread.o
OBJS_all = $(OBJS_robot_state_publisher)

HAVE_KDL = $(if $(shell $(PKGCONFIG) --exists 'orocos-kdl'; echo $${?/1/}),1,0)

ifeq ($(HAVE_KDLPARSER)$(HAVE_EIGEN3),11)
  ifeq ($(HAVE_TF),1)
    PLUGINS_all = $(PLUGINDIR)/robot-state-publisher.so
    CFLAGS += $(CFLAGS_TF) $(CFLAGS_KDLPARSER) $(CFLAGS_EIGEN3)
    LDFLAGS += $(LDFLAGS_TF) $(LDFLAGS_KDLPARSER) $(LDFLAGS_EIGEN3) \
      $(call boost-lib-lflags,scoped_ptr)
  else
    WARN_TARGETS += warning_tf
  endif
else
  WARN_TARGETS += warning_kdlparser
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_tf warning_kdlparser
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Disabling robot state publisher plugin$(TNORMAL) (tf not available)"
warning_kdlparser:
	$(SILENT)echo -e "$(INDENT_PRINT) --> $(TRED)Disabling robot state publisher plugin$(TNORMAL) (kdl_parser not available)"
endif

include $(BUILDSYSDIR)/base.mk
