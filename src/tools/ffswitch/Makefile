#*****************************************************************************
#            Makefile Build System for Fawkes : SwitchInterface Tool
#                            -------------------
#   Created on Wed Nov 22 00:29:31 2006
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

LIBS_ffswitch = stdc++ fawkescore fawkesutils fawkesnetcomm fawkesblackboard	\
		fawkesinterface SwitchInterface
OBJS_ffswitch = main.o

OBJS_all     =	$(OBJS_ffswitch)
BINS_all     =	$(BINDIR)/ffswitch
MANPAGES_all =	$(MANDIR)/man1/ffswitch.1

include $(BUILDSYSDIR)/base.mk
