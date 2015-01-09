#*****************************************************************************
#              Makefile Build System for Fawkes: NavGraph Aspect
#                            -------------------
#   Created on Tue Oct 08 18:11:35 2013
#   Copyright (C) 2006-2010 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(LIBSRCDIR)/utils/utils.mk

CFLAGS += $(CFLAGS_CPP11)

LIBS_libfawkesnavgraphaspect = fawkescore fawkesaspects fawkesutils \
			       fawkesnavgraph
OBJS_libfawkesnavgraphaspect = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))

OBJS_all    = $(OBJS_libfawkesnavgraphaspect)

ifeq ($(HAVE_CPP11),1)
  LIBS_all = $(LIBDIR)/libfawkesnavgraphaspect.$(SOEXT)
else
  ifneq ($(HAVE_TF),1)
    WARN_TARGETS = warning_tf
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_cpp11
warning_cpp11:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting navgraph plugin$(TNORMAL) (C++11 support required)"
endif


include $(BUILDSYSDIR)/base.mk

