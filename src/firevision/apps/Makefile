#*****************************************************************************
#               Makefile Build System for Fawkes: FireVision
#                            -------------------
#   Created on Sun Jan 14 18:24:42 2007
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

BASEDIR = ../../..

SUBDIRS =

include $(BASEDIR)/etc/buildsys/config.mk
include $(BASEDIR)/src/firevision/fvconf.mk

ifeq ($(BUILD_TYPE),rcsoftx)
# Stuff to be built for RCSoftX only
SUBDIRS += cannikin geegaw fountain
else
SUBDIRS += base
endif

include $(BASEDIR)/etc/buildsys/rules.mk

