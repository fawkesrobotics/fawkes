#*****************************************************************************
#               Makefile Build System for Fawkes: Config QA
#                            -------------------
#   Created on Thu Jan 11 15:38:35 2007
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

BASEDIR = ../../../..

CFLAGS = -g

OBJS_qa_config = qa_config.o
LIBS_qa_config = fawkesconfig

OBJS_qa_config_change_handler = qa_config_change_handler.o
LIBS_qa_config_change_handler = fawkesconfig

OBJS_qa_config_net_list_content = qa_net_list_content.o
LIBS_qa_config_net_list_content = fawkesconfig

OBJS_all = $(OBJS_qa_config) $(OBJS_qa_config_change_handler) \
	$(OBJS_qa_config_net_list_content)
BINS_all = $(BINDIR)/qa_config 				\
	$(BINDIR)/qa_config_change_handler 		\
	$(BINDIR)/qa_config_net_list_content

include $(BASEDIR)/etc/buildsys/base.mk

