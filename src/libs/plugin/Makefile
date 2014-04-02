#*****************************************************************************
#              Makefile Build System for Fawkes: Utility Library
#                            -------------------
#   Created on Sun Sep 03 14:14:14 2006
#   Copyright (C) 2006-2008 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(LIBSRCDIR)/utils/utils.mk

ifneq ($(wildcard $(SYSROOT)/usr/include/libelf.h),)
  HAVE_LIBELF=1
  CFLAGS += -DHAVE_LIBELF
else
  ERROR_TARGETS += error_libelf
endif

LIBS_libfawkesplugin = stdc++ elf fawkescore fawkesutils fawkesconfig fawkesnetcomm \
			fawkeslogging $(if $(filter Linux,$(OS)),dl)
OBJS_libfawkesplugin =	$(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp $(SRCDIR)/*/*.cpp)))))
HDRS_libfawkesplugin = $(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.h $(SRCDIR)/*/*.h))

OBJS_all = $(OBJS_libfawkesplugin)

ifeq ($(HAVE_LIBELF),1)
  LIBS_all  = $(LIBDIR)/libfawkesplugin.so
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(ERROR_TARGETS)
.PHONY: error_libelf
error_libelf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Fawkes plugin library cannot be built$(TNORMAL) (libelf not found)"
	$(SILENT)exit 1
endif

include $(BUILDSYSDIR)/base.mk

