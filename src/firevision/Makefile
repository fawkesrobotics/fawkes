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

BASEDIR = ../..

SUBDIRS = fvutils cams models filters classifiers stereo apps

include $(BASEDIR)/etc/buildsys/config.mk
include $(BASEDIR)/etc/buildsys/rules.mk
include $(BASEDIR)/src/firevision/fvconf.mk

cams models filters classifiers: fvutils
filters: models
apps: fvutils models cams filters classifiers stereo

