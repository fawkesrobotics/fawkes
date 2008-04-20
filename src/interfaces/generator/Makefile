#*****************************************************************************
#            Makefile Build System for Fawkes : Interface Generator
#                            -------------------
#   Created on Fri Feb 01 11:37:25 2008
#   copyright (C) 2006-2008 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

ifneq ($(wildcard ../../../etc/buildsys/config.mk),)
  # Fawkes
  BASEDIR = ../../..
else
  # RCSoftX
  BASEDIR = ../../../..
endif

include $(BASEDIR)/etc/buildsys/config.mk

CFLAGS += -DINTERFACEDIR=\"$(INTERFACEDIR)\" \
          $(shell $(PKGCONFIG) --cflags libxml++-2.6)

include $(SRCDIR)/generator_rules.mk
include $(BASEDIR)/etc/buildsys/base.mk

