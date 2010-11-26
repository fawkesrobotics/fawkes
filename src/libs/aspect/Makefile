#*****************************************************************************
#                 Makefile Build System for Fawkes: Aspects
#                            -------------------
#   Created on Thu Jan 11 16:41:34 2007
#   Copyright (C) 2006-2007 by Tim Niemueller, AllemaniACs RoboCup Team
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

ifeq ($(HAVE_FIREVISION),1)
  INCDIRS += $(realpath $(BASEDIR)/src/firevision)
else
  OBJS_FILTER_OUT += $(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/vision*.cpp))
endif

LIBS_libfawkesaspects = fawkescore fawkesutils fawkesblackboard fawkesnetcomm fawkesconfig fawkesplugin fawkeswebview
OBJS_libfawkesaspects = $(patsubst %.cpp,%.o,$(filter-out $(OBJS_FILTER_OUT),$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp $(SRCDIR)/*/*.cpp))))))
HDRS_libfawkesaspects = $(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.h $(SRCDIR)/*/*.h))
INST_HDRS_SUBDIR_libfawkesaspects = aspect

OBJS_all = $(OBJS_libfawkesaspects)
LIBS_all  = $(LIBDIR)/libfawkesaspects.so

include $(BUILDSYSDIR)/base.mk

