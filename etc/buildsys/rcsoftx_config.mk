#*****************************************************************************
#     Makefile Build System for Fawkes: Config Settings specific to RCSoftX
#                            -------------------
#   Created on Tue Apr 10 15:29:29 2007
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
#
#           $Id$
# last modified: $Date$
#            by: $Author$
#
#*****************************************************************************

# External in RCSoftX
DEFAULT_INCLUDES = -I$(BASEDIR)/src -I$(BASEDIR)/src/fawkes -I$(BASEDIR)/src/libs
LIBDIR = $(abspath $(BASEDIR)/lib/fawkes)

CONFIG_DIR ?= $(SRCDIR)
GENCONFIG_DIR = $(BASEDIR)/src/modules/tools/genconfig
GENCONFIG = $(GENCONFIG_DIR)/genconfig
GENCONFIG_CONFIG = $(CONFIG_DIR)/config.tmpl
ifeq ($(realpath $(CONFIG_DIR)/config.template.cpp),)
GENCONFIG_TEMPLATE_CPP = $(GENCONFIG_DIR)/config.template.cpp
else
GENCONFIG_TEMPLATE_CPP = $(CONFIG_DIR)/config.template.cpp
endif
ifeq ($(realpath $(CONFIG_DIR)/config.template.h),)
GENCONFIG_TEMPLATE_H = $(GENCONFIG_DIR)/config.template.h
else
GENCONFIG_TEMPLATE_H = $(CONFIG_DIR)/config.template.h
endif

