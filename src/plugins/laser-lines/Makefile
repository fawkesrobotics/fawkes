#*****************************************************************************
#         Makefile Build System for Fawkes: Laser Lines Plugin
#                            -------------------
#   Created on Fri May 23 18:09:30 2014
#   Copyright (C) 2014 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BUILDSYSDIR)/pcl.mk
include $(BUILDSYSDIR)/ros.mk
include $(BUILDSYSDIR)/boost.mk
include $(BUILDCONFDIR)/tf/tf.mk

PRESUBDIRS = interfaces

REQUIRED_PCL_LIBS = sample_consensus segmentation filters surface search

ifeq ($(CC),gcc)
  ifneq ($(call gcc_atleast_version,4,6),1)
    GCC_TOO_OLD=1
  endif
endif

CFLAGS += $(CFLAGS_CPP11) -Wno-unknown-pragmas

# Enable for time measurements
#CFLAGS += -DUSE_TIMETRACKER

LIBS_laser_lines = fawkescore fawkesutils fawkesaspects fvutils fawkesbaseapp \
			fawkestf fawkesinterface fawkesblackboard fawkespcl_utils \
			Position3DInterface SwitchInterface LaserLineInterface
OBJS_laser_lines = laser-lines-plugin.o laser-lines-thread.o  line_info.o

OBJS_all    = $(OBJS_laser_lines)

ifneq ($(GCC_TOO_OLD),1)
  ifeq ($(HAVE_PCL)$(HAVE_TF),11)
    ifeq ($(call pcl-have-libs,$(REQUIRED_PCL_LIBS)),1)
      CFLAGS += $(CFLAGS_TF) $(CFLAGS_PCL) \
	        $(call pcl-libs-cflags,$(REQUIRED_PCL_LIBS)) \
	        -Wno-deprecated -D_FILE_OFFSET_BITS=64 -D_LARGE_FILES

      LDFLAGS += $(LDFLAGS_TF) $(LDFLAGS_PCL) \
	         $(call pcl-libs-ldflags,$(REQUIRED_PCL_LIBS))

      ifeq ($(HAVE_ROS),1)
        ifeq ($(call ros-have-pkg,visualization_msgs),1)
	  LIBS_laser_lines += fawkesrosaspect
          CFLAGS  += -DHAVE_VISUAL_DEBUGGING $(CFLAGS_ROS) \
		     $(CFLAGS_EIGEN3) $(call ros-pkg-cflags,visualization_msgs)
          LDFLAGS += $(LDFLAGS_ROS) $(call ros-pkg-lflags,visualization_msgs)
        endif
      endif

      PLUGINS_all = $(PLUGINDIR)/laser-lines.$(SOEXT)
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

CFLAGS  += $(call boost-lib-cflags,thread)
LDFLAGS += $(call boost-lib-ldflags,thread)

# Required for weirdo PCL (on Fedora with ROS headers)
CFLAGS  += $(call boost-lib-cflags,system)
LDFLAGS += $(call boost-lib-ldflags,system)


ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_tf warning_pcl warning_ros warning_visualization_msgs warning_old_gcc
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting laser-lines plugin$(TNORMAL) (fawkestf not available)"
warning_pcl:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting laser-lines plugin$(TNORMAL) (PCL not available)"
warning_pcl_components:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting laser-lines plugin$(TNORMAL) (missing PCL components: $(call pcl-missing-libs,$(REQUIRED_PCL_LIBS)))"
warning_old_gcc:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting laser-lines plugin$(TNORMAL) (GCC too old, have $(GCC_VERSION), required 4.6)"
endif

include $(BUILDSYSDIR)/base.mk

