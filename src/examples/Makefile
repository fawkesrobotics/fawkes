#*****************************************************************************
#                  Makefile for Fawkes Build System: examples
#                            -------------------
#   Created on Tue Sep 12 13:28:19 2006
#   Copyright (C) 2006 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

BASEDIR = ../..

OBJS_example_mutex_count = example_mutex_count.o
LIBS_example_mutex_count = stdc++ fawkescore

OBJS_example_mutex_sync = example_mutex_sync.o
LIBS_example_mutex_sync = stdc++ fawkescore

OBJS_example_waitcond = example_waitcond.o
LIBS_example_waitcond = stdc++ fawkescore

OBJS_example_waitcond_serialize = example_waitcond_serialize.o
LIBS_example_waitcond_serialize = stdc++ fawkescore

OBJS_example_rwlock = example_rwlock.o
LIBS_example_rwlock = stdc++ fawkescore

OBJS_example_barrier = example_barrier.o
LIBS_example_barrier = stdc++ fawkescore

OBJS_example_exception = example_exception.o
LIBS_example_exception = stdc++ fawkescore

OBJS_example_refcount = example_refcount.o
LIBS_example_refcount = stdc++ fawkescore


OBJS_all =	$(OBJS_example_mutex_count)	\
		$(OBJS_example_mutex_sync)	\
		$(OBJS_example_wait_condition)	\
		$(OBJS_example_waitcond)	\
		$(OBJS_example_waitcond_serialize)	\
		$(OBJS_example_rwlock)		\
		$(OBJS_example_barrier)		\
		$(OBJS_example_exception)	\
		$(OBJS_example_refcount)

BINS_all =	$(BINDIR)/example_mutex_count		\
		$(BINDIR)/example_waitcond		\
		$(BINDIR)/example_waitcond_serialize	\
		$(BINDIR)/example_rwlock		\
		$(BINDIR)/example_barrier		\
		$(BINDIR)/example_exception		\
		$(BINDIR)/example_mutex_sync		\
		$(BINDIR)/example_refcount

include $(BASEDIR)/etc/buildsys/base.mk

