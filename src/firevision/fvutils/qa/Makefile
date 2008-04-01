#*****************************************************************************
#           Makefile Build System for Fawkes : FireVision Utils QA
#                            -------------------
#   Created on Wed Apr 11 16:00:32 2007
#   Copyright (C) 2006-2007 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BASEDIR)/src/firevision/fvconf.mk

CFLAGS   += $(VISION_CFLAGS)
LDFLAGS  += $(VISION_LDFLAGS)
INCDIRS  += $(VISION_INCDIRS)
LIBDIRS  += $(VISION_LIBDIRS)
LIBS     += $(VISION_LIBS)

OBJS_fv_qa_camargp := qa_camargp.o
LIBS_fv_qa_camargp := fvutils

OBJS_fv_qa_jpegbm := qa_jpegbm.o
LIBS_fv_qa_jpegbm := fvutils

OBJS_fv_qa_shmimg := qa_shmimg.o
LIBS_fv_qa_shmimg := fvutils

OBJS_fv_qa_rectlut := qa_rectlut.o
LIBS_fv_qa_rectlut := fvutils

OBJS_fv_qa_fuse := qa_fuse.o
LIBS_fv_qa_fuse := fvutils

OBJS_fv_qa_shmlut := qa_shmlut.o
LIBS_fv_qa_shmlut := fvutils

OBJS_fv_qa_colormap := qa_colormap.o
LIBS_fv_qa_colormap := fvutils fvwidgets

OBJS_all = $(OBJS_fv_qa_camargp)		\
	   $(OBJS_fv_qa_jpegbm)			\
	   $(OBJS_fv_qa_shmimg)			\
	   $(OBJS_fv_qa_shmlut)			\
	   $(OBJS_fv_qa_rectlut)		\
	   $(OBJS_fv_qa_fuse)			\
	   $(OBJS_fv_qa_colormap)
BINS_all = $(BINDIR)/fv_qa_camargp		\
	   $(BINDIR)/fv_qa_jpegbm		\
	   $(BINDIR)/fv_qa_shmimg		\
	   $(BINDIR)/fv_qa_shmlut		\
	   $(BINDIR)/fv_qa_rectlut		\
	   $(BINDIR)/fv_qa_fuse			\
	   $(BINDIR)/fv_qa_colormap

include $(BASEDIR)/etc/buildsys/base.mk

