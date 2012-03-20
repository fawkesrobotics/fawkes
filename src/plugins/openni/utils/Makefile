#*****************************************************************************
#                Makefile Build System for Fawkes: OpenNI Utils
#                            -------------------
#   Created on Thu Mar 24 10:24:37 2011
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
include $(SRCDIR)/../openni.mk

CFLAGS  += $(CFLAGS_OPENNI)

LIBS_libfawkesopenni_client_utils = stdc++ m fawkescore fawkesaspects fawkesutils \
			    fawkesblackboard fawkesinterface \
			    HumanSkeletonInterface ObjectPositionInterface \
			    HumanSkeletonProjectionInterface
OBJS_libfawkesopenni_client_utils = colors.o hand_if_observer.o skel_if_observer.o

LIBS_libfawkesopenni_utils = stdc++ m fawkescore fawkesaspects fawkesutils
LDFLAGS_libfawkesopenni_utils += $(LDFLAGS_OPENNI)
OBJS_libfawkesopenni_utils = setup.o conversions.o


OBJS_all = $(OBJS_libfawkesopenni_utils) $(OBJS_libfawkesopenni_client_utils)
LIBS_all = $(LIBDIR)/libfawkesopenni_client_utils.so

ifeq ($(HAVE_OPENNI),1)
  LIBS_all += $(LIBDIR)/libfawkesopenni_utils.so
else
  WARN_TARGETS += warning_openni
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_openni
warning_openni:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Partially omitting OpenNI Utils$(TNORMAL) (openni[-devel] not installed)"
endif

include $(BUILDSYSDIR)/base.mk

