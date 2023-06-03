#*****************************************************************************
#   Makefile Build System for Fawkes : Protobuf Messages for metrics plugin
#                            -------------------
#   Created on Sat May 06 19:40:30 2017
#   Copyright (C) 2017 by Tim Niemueller, Carologistics RoboCup Team
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

PROTOBUF_all = metrics_msgs

MSGS_metrics_msgs = $(notdir $(patsubst %.proto,%,$(wildcard $(SRCDIR)/*.proto)))

include $(BUILDSYSDIR)/protobuf.mk
include $(BUILDSYSDIR)/base.mk
