#*****************************************************************************
#          Makefile for Fawkes Transforms Library (based on ROS tf)
#                            -------------------
#   Created on Tue Oct 18 16:14:13 2011
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

LIBS_libfawkestf = fawkescore fawkesutils fawkesblackboard fawkesinterface \
		   TransformInterface
OBJS_libfawkestf = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.cpp $(SRCDIR)/*/*.cpp $(SRCDIR)/*/*/*.cpp))))
HDRS_libfawkestf = $(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.h $(SRCDIR)/*/*.h  $(SRCDIR)/*/*/*.h ))

OBJS_all = $(OBJS_libfawkestf)

ifeq ($(HAVE_TF),1)
  CFLAGS  += $(CFLAGS_TF)
  LDFLAGS += $(LDFLAGS_TF)
  LIBS_all = $(LIBDIR)/libfawkestf.so
else
  WARN_TARGETS += warning_bullet
endif

ifeq ($(OBJSSUBMAKE),1)
  ifneq ($(WARN_TARGETS),)
all: $(WARN_TARGETS)
  endif
.PHONY: warning_bullet
warning_bullet:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting transforms library$(TNORMAL) (bullet[-devel] not installed)"
endif

include $(BUILDSYSDIR)/base.mk
