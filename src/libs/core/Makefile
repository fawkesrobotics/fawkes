#*****************************************************************************
#                      Makefile for Fawkes Core Library
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

BASEDIR = ../../..

CFLAGS = -g

LIBS_libcore = pthread stdc++
OBJS_libcore =	\
		exception.o			\
		plugin.o			\
		exceptions/software.o		\
		exceptions/system.o		\
		threading/barrier.o		\
		threading/mutex.o		\
		threading/read_write_lock.o	\
		threading/refc_rwlock.o		\
		threading/thread.o		\
		threading/thread_collector.o	\
		threading/thread_initializer.o	\
		threading/thread_list.o		\
		threading/wait_condition.o	\
		utils/refcount.o

OBJS_all = $(OBJS_libcore)
LIBS_all  = $(LIBDIR)/libcore.so

include $(BASEDIR)/etc/buildsys/base.mk

