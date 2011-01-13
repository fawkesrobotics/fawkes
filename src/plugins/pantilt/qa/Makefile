#*****************************************************************************
#           Makefile Build System for Fawkes : Robotis PTU QA
#                            -------------------
#   Created on Thu Jun 18 10:17:05 2009
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

BASEDIR = ../../../..
include $(BASEDIR)/etc/buildsys/config.mk

OBJS_qa_rx28ptu := qa_rx28ptu.o ../robotis/rx28.o
LIBS_qa_rx28ptu := fawkescore fawkesutils

OBJS_qa_evid100p := qa_evid100p.o ../sony/visca.o ../sony/evid100p.o
LIBS_qa_evid100p := m fawkescore fawkesutils

OBJS_all = $(OBJS_qa_rx28ptu) $(OBJS_qa_evid100p)
BINS_all = $(BINDIR)/qa_rx28ptu $(BINDIR)/qa_evid100p

include $(BUILDSYSDIR)/base.mk
