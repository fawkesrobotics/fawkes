#*****************************************************************************
#                 Makefile Build System for Fawkes: Aspects
#                            -------------------
#   Created on Thu Jan 11 16:41:34 2007
#   Copyright (C) 2006-2007 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BUILDCONFDIR)/webview/webview.mk
include $(BUILDCONFDIR)/tf/tf.mk
include $(BUILDSYSDIR)/pcl.mk

LIBS_libfawkesaspects = fawkescore fawkesutils fawkesblackboard fawkesnetcomm \
			fawkesconfig fawkesplugin fawkeslogging fawkessyncpoint

ifneq ($(HAVE_WEBVIEW),1)
  OBJS_FILTER_OUT += $(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/webview.cpp $(SRCDIR)/inifins/webview.cpp))
  WARN_TARGETS += warning_webview
else
  CFLAGS += -DHAVE_WEBVIEW
  LIBS_libfawkesaspects += fawkeswebview
endif

ifneq ($(HAVE_TF),1)
  OBJS_FILTER_OUT += $(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/tf.cpp $(SRCDIR)/inifins/tf.cpp))
  WARN_TARGETS += warning_tf
else
  CFLAGS  += $(CFLAGS_TF)
  LDFLAGS += $(LDFLAGS_TF)
  LIBS_libfawkesaspects += fawkestf
endif

ifneq ($(HAVE_PCL),1)
  OBJS_FILTER_OUT += $(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/pointcloud.cpp \
		     $(SRCDIR)/inifins/pointcloud.cpp $(SRCDIR)/pointcloud/pointcloud_manager.cpp))
  WARN_TARGETS += warning_pcl
else
  LIBS_libfawkesaspects += fawkespcl_utils
  CFLAGS  += $(CFLAGS_PCL)
  LDFLAGS += $(LDFLAGS_PCL)
endif

OBJS_libfawkesaspects = $(patsubst %.cpp,%.o,$(filter-out $(OBJS_FILTER_OUT),$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp $(SRCDIR)/*/*.cpp))))))
HDRS_libfawkesaspects = $(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.h $(SRCDIR)/*/*.h))
INST_HDRS_SUBDIR_libfawkesaspects = aspect

OBJS_all = $(OBJS_libfawkesaspects)
LIBS_all  = $(LIBDIR)/libfawkesaspects.so

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_webview warning_tf warning_pcl
warning_webview:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Excluding WebviewAspect$(TNORMAL) (libfawkeswebview not built)"
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Excluding TransformAspect$(TNORMAL) (libfawkestf not built)"
warning_pcl:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Excluding PointCloudAspect$(TNORMAL) (PCL not available)"
endif

include $(BUILDSYSDIR)/base.mk

