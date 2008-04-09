#*****************************************************************************
#                 Makefile Build System for Fawkes: Navigator Library
#                            -------------------
#   Created on Fri Jun 29 16:42:05 2007
#   Copyright (C) 2008 by Daniel Beck, AllemaniACs RoboCup Team
#
#   $Id$
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

LIBS_librobotmotion = geometry utils
OBJS_librobotmotion = motion_model.o 			\
		      omni_motion_model.o 		\
		      velocity_controller.o		\
		      linear_velocity_controller.o	\
		      pid_controller.o

OBJS_all    = $(OBJS_librobotmotion)
LIBS_all    = $(LIBDIR)/librobotmotion.so

include $(BASEDIR)/etc/buildsys/base.mk
