#*****************************************************************************
#           Makefile Build System for Fawkes : FireVision Utils QA
#                            -------------------
#   Created on Wed Apr 11 16:00:32 2007
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

BASEDIR = ../../../..
include $(BASEDIR)/src/firevision/fvconf.mk

CFLAGS   = $(VISION_CFLAGS)
LDFLAGS  = $(VISION_LDFLAGS)
INCDIRS  = $(VISION_INCDIRS)
LIBDIRS  = $(VISION_LIBDIRS)
LIBS     = $(VISION_LIBS)

OBJS_fv_qa_camargp := qa_camargp.o
LIBS_fv_qa_camargp := fvutils

OBJS_fv_qa_yuvconv := qa_yuvconv.o
LIBS_fv_qa_yuvconv := fvutils

OBJS_all = $(OBJS_fv_qa_camargp) $(OBJS_fv_qa_yuvconv)
BINS_all = $(BINDIR)/fv_qa_camargp	\
	   $(BINDIR)/fv_qa_yuvconv

include $(BASEDIR)/etc/buildsys/base.mk

