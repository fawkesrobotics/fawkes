#*****************************************************************************
#             Makefile Build System for Fawkes : RefBox Repeater
#                            -------------------
#   Created on Wed Apr 09 09:44:15 2008
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

LIBS_ffrefboxrep = fawkescore fawkesutils fawkesconfig fawkesnetcomm fawkesblackboard GameStateInterface
#OBJS_ffrefboxrep = main.o refbox_state_sender.o midsize.o spl.o
OBJS_ffrefboxrep = main.o refbox_state_writer.o refbox_state_sender.o msl2007.o msl2008.o msl2010.o spl.o

OBJS_all =	$(OBJS_ffrefboxrep)
BINS_all =	$(BINDIR)/ffrefboxrep

CFLAGS  += $(shell $(PKGCONFIG) --cflags libxml++-2.6)
LDFLAGS += $(shell $(PKGCONFIG) --libs libxml++-2.6)

include $(BUILDSYSDIR)/base.mk
