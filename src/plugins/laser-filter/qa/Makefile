#*****************************************************************************
#         Makefile Build System for Fawkes : Laser Hough Transform QA
#                            -------------------
#   Created on Wed Dec 30 12:00:00 2009
#   Copyright (C) 2006-2009 by Tim Niemueller, AllemaniACs RoboCup Team
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

OBJS_qa_projection := qa_projection.o ../filters/filter.o ../filters/projection.o
LIBS_qa_projection := fawkescore fawkesutils fawkesgeometry m

OBJS_all = $(OBJS_qa_projection)
BINS_all = $(BINDIR)/qa_projection

include $(BUILDSYSDIR)/base.mk
