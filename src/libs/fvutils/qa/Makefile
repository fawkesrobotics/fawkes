
#           Makefile Build System for Fawkes : FireVision Utils QA
#                            -------------------
#   Created on Wed Apr 11 16:00:32 2007
#   Copyright (C) 2006-2007 by Tim Niemueller, AllemaniACs RoboCup Team
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

include $(BASEDIR)/etc/buildsys/config.mk
include $(BUILDSYSDIR)/fvconf.mk

CFLAGS   += $(VISION_CFLAGS)
LDFLAGS  += $(VISION_LDFLAGS)
INCDIRS  += $(VISION_INCDIRS)
LIBDIRS  += $(VISION_LIBDIRS)
LIBS     += $(VISION_LIBS)

OBJS_fv_qa_camargp := qa_camargp.o
LIBS_fv_qa_camargp := fvutils

OBJS_fv_qa_jpegbm := qa_jpegbm.o
LIBS_fv_qa_jpegbm := fvutils fawkesutils

OBJS_fv_qa_shmimg := qa_shmimg.o
LIBS_fv_qa_shmimg := fvutils fawkesutils

OBJS_fv_qa_rectlut := qa_rectlut.o
LIBS_fv_qa_rectlut := fvutils

OBJS_fv_qa_fuse := qa_fuse.o
LIBS_fv_qa_fuse := fvutils fawkescore

OBJS_fv_qa_shmlut := qa_shmlut.o
LIBS_fv_qa_shmlut := fvutils fawkesutils

OBJS_fv_qa_colormap := qa_colormap.o
LIBS_fv_qa_colormap := fvutils fvwidgets fawkescore

OBJS_fv_qa_createimage := qa_createimage.o
LIBS_fv_qa_createimage := fvutils

#ifneq ($(wildcard $(FVBASEDIR)/fvutils/recognition/forest/forest.h),)
#  OBJS_fv_qa_randomtree := qa_randomtree.o
#  LIBS_fv_qa_randomtree := fvutils
#  OBJS_all += $(OBJS_fv_qa_randomtree)
#  BINS_all += $(BINDIR)/fv_qa_randomtree
#endif

OBJS_all += $(OBJS_fv_qa_camargp)		\
	    $(OBJS_fv_qa_jpegbm)		\
	    $(OBJS_fv_qa_shmimg)		\
	    $(OBJS_fv_qa_shmlut)		\
	    $(OBJS_fv_qa_rectlut)		\
	    $(OBJS_fv_qa_fuse)			\
	    $(OBJS_fv_qa_createimage)		\
	    $(OBJS_fv_qa_colormap)


BINS_all += $(BINDIR)/fv_qa_camargp		\
	    $(BINDIR)/fv_qa_jpegbm		\
	    $(BINDIR)/fv_qa_shmimg		\
	    $(BINDIR)/fv_qa_shmlut		\
	    $(BINDIR)/fv_qa_rectlut		\
	    $(BINDIR)/fv_qa_fuse		\
	    $(BINDIR)/fv_qa_createimage

BINS_gui += $(BINDIR)/fv_qa_colormap

include $(BUILDSYSDIR)/base.mk

