#*****************************************************************************
#         Makefile Build System for Fawkes : FireVision Filters QA
#                            -------------------
#   Created on Mon May 21 17:33:45 2012
#   Copyright (C) 2006-2012 by Tim Niemueller, AllemaniACs RoboCup Team
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

CFLAGS   += $(VISION_CFLAGS) $(CFLAGS_OPENCV)
LDFLAGS  += $(VISION_LDFLAGS) $(LDFLAGS_OPENCV)
INCDIRS  += $(VISION_INCDIRS)
LIBDIRS  += $(VISION_LIBDIRS)
LIBS     += $(VISION_LIBS)

OBJS_fv_qa_sobel := qa_sobel.o
LIBS_fv_qa_sobel := fvutils fvwidgets fvfilters fvcams fawkesutils

OBJS_fv_qa_gauss := qa_gauss.o
LIBS_fv_qa_gauss := fvutils fvwidgets fvfilters fvcams fawkesutils

OBJS_fv_qa_sharpen := qa_sharpen.o
LIBS_fv_qa_sharpen := fvutils fvwidgets fvfilters fvcams fawkesutils

OBJS_fv_qa_erode := qa_erode.o
LIBS_fv_qa_erode := fvutils fvwidgets fvfilters fvcams fawkesutils


OBJS_all = $(OBJS_fv_qa_sobel) $(OBJS_fv_qa_gauss) $(OBJS_fv_qa_sharpen) \
	   $(OBJS_fv_qa_erode)

ifneq ($(HAVE_OPENCV)$(HAVE_IPP),00)
  BINS_all = $(BINDIR)/fv_qa_sobel $(BINDIR)/fv_qa_gauss \
	      $(BINDIR)/fv_qa_sharpen $(BINDIR)/fv_qa_erode
endif

include $(BUILDSYSDIR)/base.mk
