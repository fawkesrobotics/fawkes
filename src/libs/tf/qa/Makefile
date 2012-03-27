#*****************************************************************************
#               Makefile for Fawkes Transforms Library QA
#                            -------------------
#   Created on Thu Oct 20 18:20:18 2011
#   Copyright (C) 2011 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BUILDCONFDIR)/tf/tf.mk

CFLAGS  += $(CFLAGS_TF)
LDFLAGS += $(LDFLAGS_TF)

LIBS_qa_tf_transformer = m fawkescore fawkesutils fawkestf
OBJS_qa_tf_transformer = qa_tf_transformer.o

OBJS_all = $(OBJS_qa_tf_transformer)
BINS_all = $(BINDIR)/qa_tf_transformer

include $(BUILDSYSDIR)/base.mk
