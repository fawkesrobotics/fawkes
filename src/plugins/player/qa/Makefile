#*****************************************************************************
#           Makefile Build System for Fawkes: Player Plugin QA
#                            -------------------
#   Created on Tue Sep 30 23:37:44 2008
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

BASEDIR = ../../../..
include $(BASEDIR)/etc/buildsys/config.mk

LIBS_qa_player_setmotor = fawkescore fawkesutils fawkesblackboard MotorInterface
OBJS_qa_player_setmotor = qa_player_setmotor.o
BINS_all = $(BINDIR)/qa_player_setmotor

OBJS_all = $(OBJS_qa_player_setmotor)

include $(BUILDSYSDIR)/base.mk
