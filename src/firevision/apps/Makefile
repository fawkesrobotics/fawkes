#*****************************************************************************
#               Makefile Build System for Fawkes: FireVision
#                            -------------------
#   Created on Sun Jan 14 18:24:42 2007
#   Copyright (C) 2006-2007 by Tim Niemueller, AllemaniACs RoboCup Team
#
#   $Id$
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

BASEDIR = ../../..

SUBDIRS =

include $(BASEDIR)/etc/buildsys/config.mk
include $(BASEDIR)/src/firevision/fvconf.mk

ifeq ($(BUILD_TYPE),rcsoftx)
  # Stuff to be built for RCSoftX only
  SUBDIRS += cannikin geegaw fountain
else
  SUBDIRS += $(subst $(CURDIR)/,,$(realpath base fountain retriever omni_ball omni_field omni_localizer))
endif

include $(BASEDIR)/etc/buildsys/rules.mk

