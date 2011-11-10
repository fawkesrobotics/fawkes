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
include $(BUILDCONFDIR)/tf/tf.mk
include $(BASEDIR)/src/plugins/openni/openni.mk

LIBS_tabletop_objects = fawkescore fawkesutils fawkesaspects fvutils \
			fawkesinterface fawkesblackboard Position3DInterface
OBJS_tabletop_objects = tabletop_objects_plugin.o tabletop_objects_thread.o

LIBS_tabletop_objects_standalone = fawkescore fvutils fvcams fawkesutils
OBJS_tabletop_objects_standalone = tabletop_objects_standalone.o

OBJS_all    = $(OBJS_tabletop_objects) $(OBJS_tabletop_objects_standalone)

ifeq ($(HAVE_PCL)$(HAVE_OPENNI)$(HAVE_TF),111)
  CFLAGS += $(CFLAGS_TF) $(CFLAGS_PCL) \
	    -Wno-deprecated -D_FILE_OFFSET_BITS=64 -D_LARGE_FILES

  LDFLAGS += $(LDFLAGS_TF) $(LDFLAGS_PCL)

  # for standalone:
  # $(CFLAGS_VTK) $(CFLAGS_OPENNI)
  # $(LDFLAGS_OPENNI) 
  #  -lpcl_io -lpcl_visualization -lboost_thread-mt 
  #  -lvtkCommon -lvtkRendering -lvtkFiltering

  PLUGINS_all = $(PLUGINDIR)/tabletop-objects.$(SOEXT)
  #BINS_all = $(BINDIR)/tabletop-objects-standalone
else
  ifneq ($(HAVE_PCL),1)
    WARN_TARGETS += warning_pcl
  endif
  ifneq ($(HAVE_TF),1)
    WARN_TARGETS += warning_tf
  endif
  ifneq ($(HAVE_OPENNI),1)
    WARN_TARGETS += warning_openni
  endif
endif


ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_tf warning_pcl warning_openni
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting tabletop objects plugin$(TNORMAL) (fawkestf not available)"
warning_pcl:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting tabletop objects plugin$(TNORMAL) (PCL not available)"
warning_openni:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)tabletop objects plugin$(TNORMAL) (OpenNI not available)"
endif

include $(BUILDSYSDIR)/base.mk

