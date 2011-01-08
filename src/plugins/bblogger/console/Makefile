#*****************************************************************************
#        Makefile Build System for Fawkes : BBLogger Console Tool
#                            -------------------
#   Created on Thu Jan 21 01:31:57 2010
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

LIBS_ffbblog = stdc++ fawkescore fawkesutils fawkesblackboard fawkesinterface \
	       SwitchInterface
OBJS_ffbblog = bblog.o ../bblogfile.o

OBJS_all =	$(OBJS_ffbblog)
BINS_all =	$(BINDIR)/ffbblog
MANPAGES_all =  $(MANDIR)/man1/ffbblog.1

include $(BUILDSYSDIR)/base.mk

