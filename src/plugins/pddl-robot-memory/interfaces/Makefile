#*****************************************************************************
#              Makefile Build System for Fawkes: RobotMemoryInterface
#                            -------------------
#   Created on Sun May 01 20:25:42 2016
#   Copyright (C) 2016 Frederik Zwilling
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

INTERFACES_all = $(notdir $(patsubst %.xml,%,$(wildcard $(SRCDIR)/*.xml)))
include $(BUILDSYSDIR)/interface.mk

include $(BUILDSYSDIR)/base.mk

