
#*****************************************************************************
#              Makefile Build System for Fawkes: Colli Plugin
#                            -------------------
#   Created on Sat Jul 13 12:00:00 2013
#   Copyright (C) 2013 by AllemaniACs
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
include $(BUILDSYSDIR)/boost.mk
include $(BUILDSYSDIR)/ros.mk
include $(BUILDSYSDIR)/pcl.mk

ENABLE_VISUAL_DEBUGGING = 1

UTILS = utils/rob utils/geometry utils/occupancygrid drive_realization drive_modes search

LIBS_colli = fawkescore fawkesutils fawkesaspects fawkesblackboard fawkestf fawkespcl_utils \
             fawkesinterface fawkesbaseapp \
	     MotorInterface Laser360Interface  NavigatorInterface

OBJS_colli = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.cpp $(foreach d,$(UTILS),$(SRCDIR)/$d/*.cpp )))))

ifeq ($(HAVE_ROS),1)
  CFLAGS  += $(CFLAGS_ROS) -DHAVE_ROS
  LDFLAGS += $(LDFLAGS_ROS)

  ifeq ($(ENABLE_VISUAL_DEBUGGING),1)
    ifeq ($(call ros-have-pkg,visualization_msgs),1)
      ifeq ($(call ros-have-pkg,nav_msgs),1)
        CFLAGS  += -DHAVE_VISUAL_DEBUGGING
        CFLAGS  += $(call ros-pkg-cflags,visualization_msgs)
        CFLAGS  += $(call ros-pkg-cflags,nav_msgs)
        LDFLAGS += $(call ros-pkg-lflags,visualization_msgs)
        LDFLAGS += $(call ros-pkg-lflags,nav_msgs)
        LIBS_colli += fawkesrosaspect
      else
        ENABLE_VISUAL_DEBUGGING = 0
        WARN_TARGETS += warning_nav_msgs
      endif
    else
      ENABLE_VISUAL_DEBUGGING = 0
      WARN_TARGETS += warning_visualization_msgs
    endif
  endif
else
  ENABLE_VISUAL_DEBUGGING = 0
  WARN_TARGETS += warning_ros
endif

ifeq ($(ENABLE_VISUAL_DEBUGGING),1)
  CFLAGS  += $(call boost-lib-cflags,thread)
  LDFLAGS += $(call boost-lib-ldflags,thread)
endif

ifeq ($(HAVE_TF)$(HAVE_PCL),11)
  CFLAGS += $(CFLAGS_TF) $(CFLAGS_PCL) \
            -Wno-deprecated -D_FILE_OFFSET_BITS=64 -D_LARGE_FILES
  LDFLAGS += $(LDFLAGS_TF) $(LDFLAGS_PCL)
  PLUGINS_all = $(PLUGINDIR)/colli.$(SOEXT)
  OBJS_all    = $(OBJS_colli)
else
  ifeq ($(HAVE_TF),1)
    WARN_TARGETS += warning_tf
  endif

  ifeq ($(HAVE_PCL),1)
    WARN_TARGETS += warning_pcl
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_tf warning_pcl warning_ros warning_visualization_msgs warning_nav_msgs
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting Colli Plugin$(TNORMAL) (fawkestf not available)"
warning_pcl:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting Colli Plugin$(TNORMAL) (PCL not available)"
warning_ros:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting visual debugging$(TNORMAL) (ROS not available)"
warning_visualization_msgs:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting visual debugging$(TNORMAL) (ROS package visualization_msgs not available)"
warning_nav_msgs:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting visual debugging$(TNORMAL) (ROS package nav_msgs not available)"

endif

include $(BUILDSYSDIR)/base.mk
