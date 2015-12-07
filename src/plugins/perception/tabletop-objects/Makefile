#*****************************************************************************
#         Makefile Build System for Fawkes: Tabletop Objects Plugin
#                            -------------------
#   Created on Fri Nov 04 18:25:50 2011
#   Copyright (C) 2011 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BUILDSYSDIR)/pcl.mk
include $(BUILDSYSDIR)/ros.mk
include $(BUILDSYSDIR)/boost.mk
include $(BUILDCONFDIR)/tf/tf.mk
include $(BASEDIR)/src/plugins/openni/openni.mk

ENABLE_VISUAL_DEBUGGING = 1
USE_BASELINK_FRAME = 1

REQUIRED_PCL_LIBS = sample_consensus segmentation filters surface features

#CFLAGS += -Wno-unknown-pragmas
ifeq ($(CC),clang)
  CFLAGS += -Wno-overloaded-virtual
endif
ifeq ($(CC),gcc)
  ifneq ($(call gcc_atleast_version,4,5),1)
    GCC_TOO_OLD=1
  endif
endif

# Enable for time measurements
#CFLAGS += -DUSE_TIMETRACKER

LIBS_tabletop_objects = fawkescore fawkesutils fawkesaspects fvutils \
			fawkestf fawkesinterface fawkesblackboard fawkespcl_utils \
			Position3DInterface SwitchInterface
OBJS_tabletop_objects = tabletop_objects_plugin.o tabletop_objects_thread.o

LIBS_tabletop_objects_standalone = fawkescore fvutils fvcams fawkesutils
OBJS_tabletop_objects_standalone = tabletop_objects_standalone.o

OBJS_all    = $(OBJS_tabletop_objects) $(OBJS_tabletop_objects_standalone)

ifneq ($(CC)$(GCC_TOO_OLD),gcc1)
  ifeq ($(HAVE_PCL)$(HAVE_TF),11)
    ifeq ($(call pcl-have-libs,$(REQUIRED_PCL_LIBS)),1)
      CFLAGS += $(CFLAGS_TF) $(CFLAGS_PCL) \
	        $(call pcl-libs-cflags,$(REQUIRED_PCL_LIBS)) \
	        -Wno-deprecated -D_FILE_OFFSET_BITS=64 -D_LARGE_FILES

      LDFLAGS += $(LDFLAGS_TF) $(LDFLAGS_PCL) \
	         $(call pcl-libs-ldflags,$(REQUIRED_PCL_LIBS))

      PLUGINS_all = $(PLUGINDIR)/tabletop-objects.$(SOEXT)
    else
      WARN_TARGETS += warning_pcl_components
    endif
  else
    ifneq ($(HAVE_PCL),1)
      WARN_TARGETS += warning_pcl
    endif
    ifneq ($(HAVE_TF),1)
      WARN_TARGETS += warning_tf
    endif
  endif
else
  WARN_TARGETS += warning_old_gcc
endif

ifeq ($(USE_BASELINK_FRAME),1)
  CFLAGS += -DUSE_BASELINK_FRAME
endif

ifeq ($(ENABLE_VISUAL_DEBUGGING),1)
  ifeq ($(HAVE_ROS),1)
    ifeq ($(call ros-have-pkg,visualization_msgs),1)
      CFLAGS_visualization_thread  = -DHAVE_VISUAL_DEBUGGING $(CFLAGS_ROS) $(CFLAGS_TF) \
				     -Wno-unused-local-typedefs \
				     $(CFLAGS_EIGEN3) $(call ros-pkg-cflags,visualization_msgs)

      # Check for newer libqhull
      ifneq ($(wildcard $(SYSROOT)/usr/include/libqhull),)
        CFLAGS_visualization_thread += -DHAVE_QHULL_2011
      endif

      CFLAGS += -DHAVE_VISUAL_DEBUGGING
      LDFLAGS += $(LDFLAGS_ROS) $(call ros-pkg-lflags,visualization_msgs)

      # Link with qhull, required by PCL therefore safe to assume it's there
      LIBS_tabletop_objects += fawkesrosaspect qhull

      OBJS_tabletop_objects += visualization_thread_base.o visualization_thread.o
    else
      ENABLE_VISUAL_DEBUGGING = 0
      WARN_TARGETS += warning_visualization_msgs
    endif
  else
    ENABLE_VISUAL_DEBUGGING = 0
    WARN_TARGETS += warning_ros
  endif
endif

ifneq ($(ENABLE_VISUAL_DEBUGGING),1)
  CFLAGS  += $(call boost-lib-cflags,thread)
  LDFLAGS += $(call boost-lib-ldflags,thread)
endif

# Required for weirdo PCL (on Fedora with ROS headers)
CFLAGS  += $(call boost-lib-cflags,system)
LDFLAGS += $(call boost-lib-ldflags,system)



ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_tf warning_pcl warning_ros warning_visualization_msgs warning_old_gcc
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting tabletop objects plugin$(TNORMAL) (fawkestf not available)"
warning_pcl:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting tabletop objects plugin$(TNORMAL) (PCL not available)"
warning_pcl_components:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting tabletop objects plugin$(TNORMAL) (missing PCL components: $(call pcl-missing-libs,$(REQUIRED_PCL_LIBS)))"
warning_ros:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Disabling visual debugging$(TNORMAL) (ROS not available)"
warning_visualization_msgs:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Disabling visual debugging$(TNORMAL) (ROS package visualization_msgs not available)"
warning_old_gcc:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting tabletop objects plugin$(TNORMAL) (GCC too old, have $(GCC_VERSION), required 4.5)"
endif

include $(BUILDSYSDIR)/base.mk

