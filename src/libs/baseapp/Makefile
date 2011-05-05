#*****************************************************************************
#          Makefile Build System for Fawkes: Base Application Library
#                            -------------------
#   Created on Wed May 04 21:20:58 2011
#   Copyright (C) 2006-2011 by Tim Niemueller, AllemaniACs RoboCup Team
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

CFLAGS  += $(CFLAGS_LIBDAEMON)
LDFLAGS += $(LDFLAGS_LIBDAEMON)

LIBS_libfawkesbaseapp = stdc++ fawkescore fawkesutils fawkesconfig \
			fawkesblackboard fawkesplugin fawkesnetcomm \
			fawkesaspects

OBJS_libfawkesbaseapp =	$(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))
HDRS_libfawkesbaseapp = $(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.h))

OBJS_all = $(OBJS_libfawkesbaseapp)
LIBS_all  = $(LIBDIR)/libfawkesbaseapp.so

ifeq ($(OBJSSUBMAKE),1)
  ifneq ($(WARN_TARGETS),)
all: $(WARN_TARGETS)
  endif
.PHONY: warning_libdaemon
warning_libdaemon:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)Building without daemonizing support$(TNORMAL) (libdaemon[-devel] not installed)"
endif

include $(BUILDSYSDIR)/base.mk

