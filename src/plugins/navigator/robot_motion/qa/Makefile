#*****************************************************************************
#              Makefile Build System for Fawkes : Geometry QA
#                            -------------------
#   Created on Sun April 06 16:37:44 2008
#   copyright (C) 2008 by Daniel Beck, AllemaniACs RoboCup Team
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

BASEDIR = ../../../../..

OBJS_qa_omni_motion_model = qa_omni_motion_model.o
LIBS_qa_omni_motion_model = robotmotion

OBJS_qa_lvc = qa_lvc.o
LIBS_qa_lvc = robotmotion

OBJS_all =	$(OBJS_qa_omni_motion_model)	\
		$(OBJS_qa_lvc)

BINS_all =	$(BINDIR)/qa_omni_motion_model	\
		$(BINDIR)/qa_lvc

include $(BASEDIR)/etc/buildsys/base.mk
