#*****************************************************************************
#            Makefile Build System for Fawkes: Webview
#                            -------------------
#   Created on Fri Oct 03 14:07:13 2 2008 (19th German Unity Day)
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

CFLAGS  += $(CFLAGS_FCGI)
LDFLAGS += $(LDFLAGS_FCGI)

LIBS_webview = fawkescore fawkesutils fawkesnetcomm fawkesblackboard
OBJS_webview = fawkeswebview.o

OBJS_all =	$(OBJS_webview)
BINS_all =	$(BINDIR)/webview

include $(BASEDIR)/etc/buildsys/base.mk

