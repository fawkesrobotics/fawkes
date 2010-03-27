#*****************************************************************************
#              Makefile Build System for Fawkes: Webview Library
#                            -------------------
#   Created on Sun Aug 30 14:26:04 2009
#   Copyright (C) 2006-2009 by Tim Niemueller, AllemaniACs RoboCup Team
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

ifeq ($(HAVE_LIBMICROHTTPD),1)
  LDFLAGS += $(LDFLAGS_LIBMICROHTTPD)
  CFLAGS  += $(CFLAGS_LIBMICROHTTPD)
  LIBS_libfawkeswebview = stdc++ fawkescore fawkesutils
  OBJS_libfawkeswebview = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp $(SRCDIR)/*/*.cpp)))))
  LIBS_all  = $(LIBDIR)/libfawkeswebview.so
endif
OBJS_all = $(OBJS_libfawkeswebview)

include $(BUILDSYSDIR)/base.mk

