#*****************************************************************************
#               Makefile Build System for Fawkes: Localization-Sim Plugin
#
#   Created on Thu Aug 08 17:51:51 2013
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

BASEDIR = ../../..

include $(BASEDIR)/etc/buildsys/config.mk
include $(BUILDCONFDIR)/tf/tf.mk
include $(BASEDIR)/src/plugins/gazebo/gazebo.mk
include $(BASEDIR)/src/plugins/gazebo/protobuf.mk

LIBS_localization_sim = fawkescore fawkesutils fawkesaspects fvutils \
	       fawkestf fawkesinterface fawkesblackboard \
	       Position3DInterface \
	       fawkesgazeboaspect
OBJS_localization_sim = localization_sim_plugin.o localization_sim_thread.o

OBJS_all    = $(OBJS_localization_sim)

ifeq ($(HAVE_TF),1)
  CFLAGS  += $(CFLAGS_TF)
  LDFLAGS += $(LDFLAGS_TF)
  CFLAGS  += $(CFLAGS_GAZEBO) $(CFLAGS_PROTOBUF)
  LDFLAGS += $(LDFLAGS_GAZEBO) $(LDFLAGS_PROTOBUF) -lm $(call boost-libs-ldflags,system) -lboost_system

  PLUGINS_all = $(PLUGINDIR)/localization_sim.so
else
  WARN_TARGETS += warning_tf
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
.PHONY: warning_tf warning_ros warning_geometry_msgs
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting Localization-Sim libs" \
                         "$(TNORMAL) (tf not available)"
endif

include $(BUILDSYSDIR)/base.mk
