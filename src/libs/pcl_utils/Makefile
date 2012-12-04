#*****************************************************************************
#                     Makefile for Fawkes PCL utlities
#                            -------------------
#   Created on Sun Nov 13 13:12:39 2011
#   Copyright (C) 2011 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BUILDSYSDIR)/pcl.mk

LIBS_libfawkespcl_utils = fawkescore fawkestf
OBJS_libfawkespcl_utils = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.cpp $(SRCDIR)/*/*.cpp $(SRCDIR)/*/*/*.cpp))))
HDRS_libfawkespcl_utils = $(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.h $(SRCDIR)/*/*.h  $(SRCDIR)/*/*/*.h ))

OBJS_all = $(OBJS_libfawkespcl_utils)

ifeq ($(HAVE_PCL)$(HAVE_TF),11)
  CFLAGS  += $(CFLAGS_PCL) $(CFLAGS_TF)
  LDFLAGS += $(LDFLAGS_PCL) $(LDFLAGS_TF)
  LIBS_all = $(LIBDIR)/libfawkespcl_utils.$(SOEXT)
else
  ifneq ($(HAVE_PCL),1)
    WARN_TARGETS += warning_pcl
  endif
  ifneq ($(HAVE_TF),1)
    WARN_TARGETS += warning_tf
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
  ifneq ($(WARN_TARGETS),)
all: $(WARN_TARGETS)
  endif
.PHONY: warning_pcl warning_tf
warning_pcl:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting PCL utility library$(TNORMAL) (pcl[-devel] not installed)"
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting PCL utility library$(TNORMAL) (tf framework not available)"
endif

include $(BUILDSYSDIR)/base.mk
