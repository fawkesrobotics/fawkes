#*****************************************************************************
#         Makefile Build System for Fawkes: OpenNI Skeleton GUI
#                            -------------------
#   Created on Wed Mar 30 16:12:20 2011
#   Copyright (C) 2011 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

ifneq ($(PKGCONFIG),)
  HAVE_GL = $(if $(shell $(PKGCONFIG) --exists 'gl'; echo $${?/1/}),1,0)
endif

ifeq ($(HAVE_GL),1)
  CFLAGS_GL  = $(shell $(PKGCONFIG) --cflags 'gl') -DHAVE_GL
  LDFLAGS_GL = $(shell $(PKGCONFIG) --libs 'gl')
endif

ifneq ($(wildcard $(SYSROOT)/usr/include/GL/glut.h),)
  HAVE_GLUT = 1
  CFLAGS_GLUT = -DHAVE_GLUT
  LDFLAGS_GLUT = -lglut
endif

