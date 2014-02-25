#*****************************************************************************
#             Makefile Build System for Fawkes : Protobuf Messages
#                            -------------------
#   Created on Sun Jan 26 01:42:45 2014
#   Copyright (C) 2006-2014 by Tim Niemueller, Carologistics RoboCup Team
#			       Frederik Zwilling
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

PROTOBUF_all = gazsim_msgs

MSGS_gazsim_msgs = $(notdir $(patsubst %.proto,%,$(wildcard $(SRCDIR)/*.proto)))

include $(BUILDSYSDIR)/protobuf.mk
include $(BUILDSYSDIR)/base.mk
