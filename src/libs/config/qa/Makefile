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
include $(BASEDIR)/etc/buildsys/config.mk

CFLAGS = -g

OBJS_qa_config_sqlite = qa_config.o
LIBS_qa_config_sqlite = fawkescore fawkesconfig

OBJS_qa_config_change_handler = qa_config_change_handler.o
LIBS_qa_config_change_handler = fawkescore fawkesconfig

OBJS_qa_config_net_list_content = qa_net_list_content.o
LIBS_qa_config_net_list_content = fawkescore fawkesconfig

OBJS_qa_config_yaml = qa_yaml.o
LIBS_qa_config_yaml = fawkescore fawkesconfig

OBJS_all = $(OBJS_qa_config_sqlite) $(OBJS_qa_config_net_list_content) \
	   $(OBJS_qa_config_yaml) 
# $(OBJS_qa_config_change_handler)
BINS_all = $(BINDIR)/qa_config_sqlite 				\
	$(BINDIR)/qa_config_yaml 				\
	$(BINDIR)/qa_config_net_list_content
#	$(BINDIR)/qa_config_change_handler

include $(BUILDSYSDIR)/base.mk

