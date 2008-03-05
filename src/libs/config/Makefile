#*****************************************************************************
#           Makefile Build System for Fawkes: Configuration Utils
#                            -------------------
#   Created on Sun Jan 07 14:56:02 2007
#   copyright (C) 2006-2007 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************
#
#           $Id$
# last modified: $Date$
#            by: $Author$
#
#*****************************************************************************

BASEDIR = ../../..
include $(BASEDIR)/etc/buildsys/config.mk

ifneq ($(HAVE_SQLITE),1)
  ERROR_TARGETS += error_sqlite
else
  CFLAGS += $(shell $(PKGCONFIG) --cflags sqlite3)
  LDFLAGS_libconfig = $(shell $(PKGCONFIG) --libs sqlite3)
  LIBS_all  = $(LIBDIR)/libconfig.so
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(ERROR_TARGETS)
.PHONY: error_sqlite
error_sqlite:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)libconfig cannot be built$(TNORMAL) (SQLite 3 not installed)"
	$(SILENT)exit 1
endif


LIBS_libconfig = stdc++ core utils netcomm
OBJS_libconfig = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))

OBJS_all = $(OBJS_libconfig)

include $(BASEDIR)/etc/buildsys/base.mk

