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
include $(BUILDSYSDIR)/pcl.mk

LIBS_libfawkespclutils = fawkescore
OBJS_libfawkespclutils = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.cpp $(SRCDIR)/*/*.cpp $(SRCDIR)/*/*/*.cpp))))
HDRS_libfawkespclutils = $(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.h $(SRCDIR)/*/*.h  $(SRCDIR)/*/*/*.h ))

OBJS_all = $(OBJS_libfawkespcl_utils)

ifeq ($(HAVE_PCL),1)
  CFLAGS  += $(CFLAGS_PCL)
  LDFLAGS += $(LDFLAGS_PCL)
  # Currently there is no code to be compiled
  #LIBS_all = $(LIBDIR)/libfawkespclutils.so
else
  WARN_TARGETS += warning_pcl
endif

ifeq ($(OBJSSUBMAKE),1)
  ifneq ($(WARN_TARGETS),)
all: $(WARN_TARGETS)
  endif
.PHONY: warning_pcl
warning_bullet:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting PCL utility library$(TNORMAL) (pcl[-devel] not installed)"
endif

include $(BUILDSYSDIR)/base.mk
