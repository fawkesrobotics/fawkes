#*****************************************************************************
#                 Makefile Build System for Fawkes: OpenRAVE
#                            -------------------
#   Created on Thu Sep 16 14:50:34 2010
#   Copyright (C) 2010 by Bahram Maleki-Fard, AllemaniACs RoboCup Team
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

LIBS_libfawkesopenrave = fawkescore fawkesutils fawkesgeometry
OBJS_libfawkesopenrave = $(patsubst %.cpp,%.o,$(filter-out $(OBJS_FILTER_OUT),$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp $(SRCDIR)/*/*.cpp))))))

#Check for openRAVE
ifneq ($(PKGCONFIG),)
  HAVE_OPENRAVE := $(if $(shell $(PKGCONFIG) --exists 'openrave'; echo $${?/1/}),1,0)
endif

ifeq ($(HAVE_OPENRAVE),1)
  CFLAGS  += $(shell $(PKGCONFIG) --cflags 'openrave')
  LDFLAGS += $(shell $(PKGCONFIG) --libs 'openrave') -lopenrave-core
  LIBS_all  = $(LIBDIR)/libfawkesopenrave.so
  OBJS_all = $(OBJS_libfawkesopenrave)
else
  WARN_TARGETS += warning_openrave
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_openrave
warning_openrave:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting OpenRAVE connector library$(TNORMAL) (openrave not installed)"
endif

include $(BUILDSYSDIR)/base.mk

