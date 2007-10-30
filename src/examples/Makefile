#*****************************************************************************
#                        Makefile for Fawkes libcore
#                            -------------------
#   Created on Tue Sep 12 13:28:19 2006
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

BASEDIR = ../..

OBJS_example_mutex_count = example_mutex_count.o
LIBS_example_mutex_count = stdc++ core

OBJS_example_mutex_sync = example_mutex_sync.o
LIBS_example_mutex_sync = stdc++ core

OBJS_example_waitcond_serialize = example_waitcond_serialize.o
LIBS_example_waitcond_serialize = stdc++ core

OBJS_example_rwlock = example_rwlock.o
LIBS_example_rwlock = stdc++ core

OBJS_example_barrier = example_barrier.o
LIBS_example_barrier = stdc++ core

OBJS_example_exception = example_exception.o
LIBS_example_exception = stdc++ core

OBJS_example_refcount = example_refcount.o
LIBS_example_refcount = stdc++ core

OBJS_example_barrier_cancel = example_barrier_cancel.o
LIBS_example_barrier_cancel = stdc++ core


OBJS_all =	$(OBJS_example_mutex_count)	\
		$(OBJS_example_mutex_sync)	\
		$(OBJS_example_wait_condition)	\
		$(OBJS_example_waitcond_serialize)	\
		$(OBJS_example_rwlock)		\
		$(OBJS_example_barrier)		\
		$(OBJS_example_exception)	\
		$(OBJS_example_refcount)		\
		$(OBJS_example_barrier_cancel)

BINS_all =	$(BINDIR)/example_mutex_count		\
		$(BINDIR)/example_waitcond_serialize	\
		$(BINDIR)/example_rwlock		\
		$(BINDIR)/example_barrier		\
		$(BINDIR)/example_exception		\
		$(BINDIR)/example_mutex_sync		\
		$(BINDIR)/example_refcount		\
		$(BINDIR)/example_barrier_cancel

include $(BASEDIR)/etc/buildsys/base.mk

