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

BASEDIR = ../../../..
include $(BASEDIR)/etc/buildsys/config.mk

CFLAGS = -g

LIBS_qa_bb_memmgr = fawkescore fawkesblackboard
OBJS_qa_bb_memmgr = qa_bb_memmgr.o

LIBS_qa_bb_interface = TestInterface fawkescore fawkesblackboard fawkesinterface
OBJS_qa_bb_interface = qa_bb_interface.o

LIBS_qa_bb_buffers = TestInterface fawkescore fawkesblackboard fawkesinterface
OBJS_qa_bb_buffers = qa_bb_buffers.o

LIBS_qa_bb_messaging = TestInterface fawkescore fawkesblackboard fawkesinterface \
			fawkesutils
OBJS_qa_bb_messaging = qa_bb_messaging.o

LIBS_qa_bb_openall = TestInterface fawkescore fawkesblackboard fawkesinterface \
			fawkesutils fawkeslogging
OBJS_qa_bb_openall = qa_bb_openall.o

LIBS_qa_bb_notify = TestInterface fawkescore fawkesblackboard fawkesinterface \
			fawkesutils fawkeslogging
OBJS_qa_bb_notify = qa_bb_notify.o

LIBS_qa_bb_listall = TestInterface fawkescore fawkesblackboard fawkesinterface
OBJS_qa_bb_listall = qa_bb_listall.o

LIBS_qa_bb_remote = TestInterface fawkescore fawkesblackboard fawkesinterface \
			fawkesutils fawkesnetcomm
OBJS_qa_bb_remote = qa_bb_remote.o

LIBS_qa_bb_objpos = fawkescore fawkesblackboard fawkesinterface ObjectPositionInterface \
			fawkesutils fawkesnetcomm fawkeslogging
OBJS_qa_bb_objpos = qa_bb_objpos.o

OBJS_all =	$(OBJS_qa_bb_memmgr) 		\
		$(OBJS_qa_bb_interface)		\
		$(OBJS_qa_bb_buffers)		\
		$(OBJS_qa_bb_messaging)		\
		$(OBJS_qa_bb_openall)		\
		$(OBJS_qa_bb_notify)		\
		$(OBJS_qa_bb_listall)		\
		$(OBJS_qa_bb_remote)		\
		$(OBJS_qa_bb_objpos)

BINS_all =	$(BINDIR)/qa_bb_memmgr		\
		$(BINDIR)/qa_bb_interface	\
		$(BINDIR)/qa_bb_buffers		\
		$(BINDIR)/qa_bb_messaging	\
		$(BINDIR)/qa_bb_notify		\
		$(BINDIR)/qa_bb_openall		\
		$(BINDIR)/qa_bb_listall		\
		$(BINDIR)/qa_bb_remote		\
		$(BINDIR)/qa_bb_objpos

include $(BUILDSYSDIR)/base.mk

