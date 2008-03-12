#*****************************************************************************
#                 Makefile Build System for Fawkes: Navigator Plugin
#                            -------------------
#   Created on Tue May 29 18:41:17 2007
#   Copyright (C) 2007 by Martin Liebenberg, AllemaniACs RoboCup Team
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

LIBS_libfuzzy =
OBJS_libfuzzy =	fuzzy_controller.o \
		fuzzy_partition.o \
		fuzzy_set.o \
		mamdani_fuzzy_controller.o \
		potential_field_controller.o \
		smoothing_controller.o \
		trapezium_set.o \
		triangle_set.o

OBJS_all    = $(OBJS_libfuzzy)
LIBS_all = $(LIBDIR)/libfuzzy.so

include $(BASEDIR)/etc/buildsys/base.mk
