#*****************************************************************************
#                      Makefile Build System for Fawkes
#                            -------------------
#   Created on Sun Sep 03 14:14:14 2006
#   copyright (C) 2006 by Tim Niemueller, AllemaniACs RoboCup Team
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

ifneq ($(realpath /usr/include/magic.h),)
  HAVE_LIBMAGIC = 1
else
  ERROR_TARGETS += error_libmagic
endif

ifneq ($(SRCDIR),)
all: $(ERROR_TARGETS)
.PHONY: error_libmagic
error_libmagic:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Utils cannot be built$(TNORMAL) (install file-devel)"
	$(SILENT)exit 1
endif

LIBS_libutils = stdc++ dl core magic
OBJS_libutils =	$(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp $(SRCDIR)/*/*.cpp $(SRCDIR)/*/*/*.cpp)))))

OBJS_all = $(OBJS_libutils)
ifeq ($(HAVE_LIBMAGIC),1)
  LIBS_all  = $(LIBDIR)/libutils.so
endif

include $(BASEDIR)/etc/buildsys/base.mk

