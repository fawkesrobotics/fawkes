#*****************************************************************************
#        Makefile Build System for Fawkes : PTU Console Tool
#                            -------------------
#   Created on Thu Oct 06 16:23:37 2011
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

BASEDIR = ../../../..
include $(BASEDIR)/etc/buildsys/config.mk

LIBS_ffptu = stdc++ fawkescore fawkesutils fawkesnetcomm fawkesblackboard \
		fawkesinterface PanTiltInterface
OBJS_ffptu = ffptu.o ../robotis/rx28.o

OBJS_all =	$(OBJS_ffptu)
BINS_all = $(BINDIR)/ffptu 

include $(BUILDSYSDIR)/base.mk

