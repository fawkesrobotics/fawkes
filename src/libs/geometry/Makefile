#*****************************************************************************
#              Makefile Build System for Fawkes: Geometry Library
#                            -------------------
#   Created on Sun Oct 14 18:48:54 2007
#   copyright (C) 2006-2007 by Tim Niemueller, AllemaniACs RoboCup Team
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

BASEDIR = ../../..
include $(BASEDIR)/etc/buildsys/config.mk

SUBDIRS = gtk

LIBS_libfawkesgeometry = stdc++
OBJS_libfawkesgeometry = matrix.o		\
			 vector.o		\
			 printable.o		\
			 transformable.o	\
			 hom_transform.o	\
			 hom_coord.o		\
			 hom_point.o		\
			 hom_vector.o		\
			 hom_polar.o		\
			 hom_pose.o		\
			 bezier.o		\
			 spline.o		\
			 line_segment.o		#\
#			 hom_pose_2d.o		\

OBJS_all = $(OBJS_libfawkesgeometry)
LIBS_all = $(LIBDIR)/libfawkesgeometry.so

include $(BASEDIR)/etc/buildsys/base.mk

