#*****************************************************************************
#              Makefile Build System for Fawkes : Interfaces QA
#                            -------------------
#   Created on Tue Oct 17 13:05:54 2006
#   copyright (C) 2006 by Tim Niemueller, AllemaniACs RoboCup Team
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

CFLAGS = -g

LIBS_qa_interface_msgq = fawkescore fawkesutils TestInterface
OBJS_qa_interface_msgq = qa_interface_msgq.o

OBJS_all =	$(OBJS_qa_interface_msgq)

BINS_all =	$(BINDIR)/qa_interface_msgq

include $(BUILDSYSDIR)/base.mk

