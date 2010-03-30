#*****************************************************************************
#                  Makefile for Fawkes Build System: qa_cores
#                            -------------------
#   Created on Tue Sep 12 13:28:19 2006
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

OBJS_qa_core_mutex_count = qa_mutex_count.o
LIBS_qa_core_mutex_count = stdc++ fawkescore

OBJS_qa_core_mutex_sync = qa_mutex_sync.o
LIBS_qa_core_mutex_sync = stdc++ fawkescore

OBJS_qa_core_waitcond = qa_waitcond.o
LIBS_qa_core_waitcond = stdc++ fawkescore

OBJS_qa_core_waitcond_serialize = qa_waitcond_serialize.o
LIBS_qa_core_waitcond_serialize = stdc++ fawkescore

OBJS_qa_core_rwlock = qa_rwlock.o
LIBS_qa_core_rwlock = stdc++ fawkescore

OBJS_qa_core_barrier = qa_barrier.o
LIBS_qa_core_barrier = stdc++ fawkescore

OBJS_qa_core_exception = qa_exception.o
LIBS_qa_core_exception = stdc++ fawkescore

OBJS_all =	$(OBJS_qa_core_mutex_count)	\
		$(OBJS_qa_core_mutex_sync)	\
		$(OBJS_qa_core_wait_condition)	\
		$(OBJS_qa_core_waitcond)	\
		$(OBJS_qa_core_waitcond_serialize)	\
		$(OBJS_qa_core_rwlock)		\
		$(OBJS_qa_core_barrier)		\
		$(OBJS_qa_core_exception)

BINS_all =	$(BINDIR)/qa_core_mutex_count		\
		$(BINDIR)/qa_core_waitcond		\
		$(BINDIR)/qa_core_waitcond_serialize	\
		$(BINDIR)/qa_core_rwlock		\
		$(BINDIR)/qa_core_barrier		\
		$(BINDIR)/qa_core_exception		\
		$(BINDIR)/qa_core_mutex_sync

include $(BUILDSYSDIR)/base.mk

