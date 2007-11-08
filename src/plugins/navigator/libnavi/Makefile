#*****************************************************************************
#                 Makefile Build System for Fawkes: Navigator Library
#                            -------------------
#   Created on Fri Jun 29 13:53:45 2007
#   copyright (C) 2007 by Martin Liebenberg, AllemaniACs RoboCup Team
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


LIBS_libnavigator = core utils netcomm
OBJS_libnavigator =    npoint.o \
									obstacle.o \
										  nline.o \
										  navigator_messages.o
		 

		 

OBJS_all    = $(OBJS_navigator) $(OBJS_libnavigator)
LIBS_all    = $(LIBDIR)/libnavigator.so

include $(BASEDIR)/etc/buildsys/base.mk
