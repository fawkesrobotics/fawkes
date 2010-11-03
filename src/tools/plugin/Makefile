#*****************************************************************************
#            Makefile Build System for Fawkes : Plugin Tool
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

LIBS_ffplugin = stdc++ fawkescore fawkesutils fawkesnetcomm fawkesplugin
OBJS_ffplugin = main.o	\
		plugin_tool.o

OBJS_all     =	$(OBJS_ffplugin)
BINS_all     =	$(BINDIR)/ffplugin
MANPAGES_all =	$(MANDIR)/man1/ffplugin.1

include $(BUILDSYSDIR)/base.mk
