#*****************************************************************************
#                 Makefile Build System for Fawkes: Aspects
#                            -------------------
#   Created on Thu Jan 11 16:41:34 2007
#   copyright (C) 2006-2007 by Tim Niemueller, AllemaniACs RoboCup Team
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

LIBS_libaspects = core blackboard netcomm config
OBJS_libaspects =	\
		blackboard.o		\
		blocked_timing.o	\
		configurable.o

OBJS_all = $(OBJS_libaspects)
LIBS_all  = $(LIBDIR)/libaspects.so

include $(BASEDIR)/etc/buildsys/base.mk

