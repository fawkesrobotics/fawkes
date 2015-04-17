#*****************************************************************************
#     Makefile Build System for Fawkes: navgraph generation
#                            -------------------
#   Created on Mon Feb 09 17:39:53 2015
#   Copyright (C) 2015 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BUILDCONFDIR)/navgraph/navgraph.mk
include $(BUILDSYSDIR)/pcl.mk
include $(BUILDCONFDIR)/tf/tf.mk
include $(BUILDSYSDIR)/ros.mk
include $(BUILDSYSDIR)/cgal.mk


PRESUBDIRS = interfaces

# Set to 1 to publish some markers for map obstacle generation visualization
USE_VISUALIZATION=1

# For adding map points
REQUIRED_PCL_LIBS = sample_consensus segmentation filters surface search

LIBS_navgraph_generator = fawkescore fawkesutils fawkesaspects	\
			  fawkesinterface fawkesblackboard fawkesnavgraph \
			  fawkes_amcl_utils fawkes_amcl_map \
			  fawkesnavgraphaspect fawkesnavgraphgenerators \
			  NavGraphGeneratorInterface
OBJS_navgraph_generator = navgraph_generator_plugin.o navgraph_generator_thread.o
OBJS_all    = $(OBJS_navgraph_generator)

HAVE_PCL_COMPONENTS=$(call pcl-have-libs,$(REQUIRED_PCL_LIBS))

ifeq ($(HAVE_NAVGRAPH)$(HAVE_CPP11)$(HAVE_PCL)$(HAVE_TF)$(HAVE_CGAL),11111)
  CFLAGS += $(CFLAGS_CPP11) $(CFLAGS_TF) $(CFLAGS_PCL) \
	    $(call pcl-libs-cflags,$(REQUIRED_PCL_LIBS)) \
	    -Wno-deprecated -D_FILE_OFFSET_BITS=64 -D_LARGE_FILES

  LDFLAGS += $(LDFLAGS_TF) $(LDFLAGS_PCL) \
	     $(call pcl-libs-ldflags,$(REQUIRED_PCL_LIBS))

  PLUGINS_all = $(PLUGINDIR)/navgraph-generator.$(SOEXT)

  ifeq ($(USE_VISUALIZATION),1)
    ifeq ($(HAVE_ROS),1)
      ifeq ($(call ros-have-pkg,visualization_msgs),1)
        LIBS_navgraph_generator += fawkesrosaspect
	OBJS_navgraph_generator += visualization_thread.o
        CFLAGS  += -DHAVE_VISUALIZATION $(CFLAGS_ROS) \
	           $(call ros-pkg-cflags,visualization_msgs)
        LDFLAGS += $(LDFLAGS_ROS) $(call ros-pkg-lflags,visualization_msgs)
      else
        WARN_TARGET += warning_visualization_msgs
      endif
    else
      WARN_TARGET += warning_ros
    endif
  endif

else
  ifneq ($(HAVE_CPP11),1)
    WARN_TARGETS = warning_cpp11
  endif
  ifneq ($(HAVE_PCL),1)
    WARN_TARGETS += warning_pcl
  endif
  ifneq ($(HAVE_PCL_COMPONENTS),1)
    WARN_TARGETS += warning_pcl_components
  endif
  ifneq ($(HAVE_TF),1)
    WARN_TARGETS += warning_tf
  endif
  ifneq ($(HAVE_CGAL),1)
    WARN_TARGETS += warning_cgal
  endif
  # navgraph.mk performs its own warning
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_cpp11 warning_tf warning_pcl warning_pcl_components warning_ros warning_visualization_msgs warning_cgal
warning_cpp11:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting navgraph-generator plugin$(TNORMAL) (C++11 support required)"
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting navgraph-generator plugin$(TNORMAL) (fawkestf not available)"
warning_cgal:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting navgraph-generator plugin$(TNORMAL) (CGAL: $(CGAL_ERROR))"
warning_pcl:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting navgraph-generator plugin$(TNORMAL) (PCL not available)"
warning_pcl_components:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting navgraph-generator plugin$(TNORMAL) (missing PCL components: $(call pcl-missing-libs,$(REQUIRED_PCL_LIBS)))"
warning_ros:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)Disabling visual debugging for navgraph-generator plugin$(TNORMAL) (ROS not available)"
warning_visualization_msgs:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)Disabling visual debugging for navgraph-generator plugin$(TNORMAL) (ROS visualizatin_msgs package not available)"
endif

include $(BUILDSYSDIR)/base.mk
