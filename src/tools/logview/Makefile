#*****************************************************************************
#            Makefile Build System for Fawkes : Log Tool
#                            -------------------
#   Created on Sat Dec 15 02:02:54 2007
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

BASEDIR = ../../..

include $(BASEDIR)/etc/buildsys/config.mk

LIBS_fflogview = stdc++ fawkescore fawkesutils fawkesnetcomm fawkeslogging \
                 fawkesnetworklogger
OBJS_fflogview = main.o

OBJS_all     =	$(OBJS_fflogview)
BINS_all     =	$(BINDIR)/fflogview
MANPAGES_all =	$(MANDIR)/man1/fflogview.1

include $(BUILDSYSDIR)/base.mk
