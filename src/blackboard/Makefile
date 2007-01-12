#*****************************************************************************
#                Makefile Build System for Fawkes : BlackBoard
#                            -------------------
#   Created on Wed Oct 04 10:49:27 2006
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

BASEDIR = ../..

CFLAGS = -g
SUBDIRS = tools

LIBS_libblackboard = core utils interfaces
OBJS_libblackboard = blackboard.o		\
		     interface_manager.o	\
		     main_thread.o		\
		     memory_manager.o		\
		     message_manager.o		\
		     shmem_header.o		\
		     shmem_lister.o

OBJS_all =	$(OBJS_libblackboard) $(OBJS_blackboard)
LIBS_all =	$(LIBDIR)/libblackboard.so

include $(BASEDIR)/etc/buildsys/base.mk

