#*****************************************************************************
#              Makefile Build System for Fawkes : BlackBoard QA
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
#
#           $Id$
# last modified: $Date$
#            by: $Author$
#
#*****************************************************************************

BASEDIR = ../../../..

CFLAGS = -g

LIBS_qa_bb_memmgr = core utils blackboard
OBJS_qa_bb_memmgr = qa_bb_memmgr.o

LIBS_qa_bb_interface = core utils interfaces blackboard
OBJS_qa_bb_interface = qa_bb_interface.o

LIBS_qa_bb_messaging = core utils interfaces blackboard
OBJS_qa_bb_messaging = qa_bb_messaging.o

OBJS_all =	$(OBJS_qa_bb_memmgr) 		\
		$(OBJS_qa_bb_interface)		\
		$(OBJS_qa_bb_messaging)

BINS_all =	$(BINDIR)/qa_bb_memmgr		\
		$(BINDIR)/qa_bb_interface	\
		$(BINDIR)/qa_bb_messaging

include $(BASEDIR)/etc/buildsys/base.mk

