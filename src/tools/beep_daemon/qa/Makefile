#*****************************************************************************
#             Makefile Build System for Fawkes: Beep Daemon QA
#                            -------------------
#   Created on Sun Apr 11 22:25:47 2010
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

LIBS_qa_remote_beep = fawkescore fawkesutils fawkesblackboard SwitchInterface
OBJS_qa_remote_beep = qa_remote_beep.o
BINS_all = $(BINDIR)/qa_remote_beep

OBJS_all = $(OBJS_qa_remote_beep)

include $(BUILDSYSDIR)/base.mk
