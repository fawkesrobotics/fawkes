#*****************************************************************************
#              Makefile Build System for Fawkes : Majority Fuser QA
#                            -------------------
#   Created on Thu Oct 05 14:14:14 2006
#   copyright (C) 2006 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

BASEDIR = ../../../../..
include $(BASEDIR)/etc/buildsys/config.mk

CFLAGS = -g

LIBS_qa_objpos_majority = ObjectPositionInterface fawkesblackboard
OBJS_qa_objpos_majority = qa_objpos_majority.o

OBJS_all =	$(OBJS_qa_objpos_majority)

BINS_all =	$(BINDIR)/qa_objpos_majority

include $(BUILDSYSDIR)/base.mk

