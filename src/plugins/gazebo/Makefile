#*****************************************************************************
#              Makefile Build System for Fawkes: Gazebo Plugin
#                            -------------------
#   Created on Sun Aug 26 11:58:30 2012
#   Author  Bastian Klingen
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
include $(BASEDIR)/etc/buildsys/boost.mk
include $(BASEDIR)/src/plugins/gazebo/gazebo.mk
include $(BASEDIR)/src/plugins/gazebo/protobuf.mk

PRESUBDIRS = aspect msgs

SUBDIRS = gazsim-robotino gazsim-laser gazsim-localization gazsim-comm \
	  gazsim-vis-localization gazsim-webcam gazsim-timesource \
	  gazsim-depthcam

LIBS_gazebo = fawkescore fawkesaspects fawkesgazeboaspect
OBJS_gazebo = gazebo_plugin.o node_thread.o

OBJS_all    = $(OBJS_gazebo)

ifeq ($(HAVE_GAZEBO)$(HAVE_PROTOBUF)$(call boost-have-libs,system),111)
  CFLAGS  += $(CFLAGS_GAZEBO) $(CFLAGS_PROTOBUF)
  LDFLAGS += $(LDFLAGS_GAZEBO) $(LDFLAGS_PROTOBUF) -lm $(call boost-libs-ldflags,system)

  PLUGINS_all = $(PLUGINDIR)/gazebo.$(SOEXT)
else
  ifneq ($(HAVE_GAZEBO),1)
    WARN_TARGETS += warning_gazebo
  endif
  ifneq ($(HAVE_PROTOBUF),1)
    WARN_TARGETS += warning_protobuf
  endif
  ifneq ($(call boost-have-libs,system),1)
    WARN_TARGETS += warning_boost_system
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_gazebo warning_protobuf warning_boost_system
warning_gazebo:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting Gazebo Plugin$(TNORMAL) (Gazebo Simulator not found)"
warning_protobuf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting Gazebo Plugin$(TNORMAL) (protobuf[-devel] not installed)"
warning_boost_system:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting Gazebo Plugin$(TNORMAL) (boost-system not installed)"
endif

include $(BUILDSYSDIR)/base.mk
