#*****************************************************************************
#                  Makefile for Fawkes Build System: qa_geometry
#                            -------------------
#  Created:  Fri Feb 13 14:31:48 2009
#  Copyright 2009 Christof Rath <christof.rath@gmail.com>
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

OBJS_qa_hom_transform = qa_hom_transform.o
LIBS_qa_hom_transform = stdc++ fawkescore fawkesgeometry fawkesutils

OBJS_qa_vector = qa_vector.o
LIBS_qa_vector = stdc++ fawkescore fawkesgeometry fawkesutils


OBJS_all =	$(OBJS_qa_hom_transform) $(OBJS_qa_vector)

BINS_all =	$(BINDIR)/qa_hom_transform $(BINDIR)/qa_vector

include $(BUILDSYSDIR)/base.mk

