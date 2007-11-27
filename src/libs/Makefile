#*****************************************************************************
#                  Makefile Build System for Fawkes: Libraries
#                            -------------------
#   Created on Tue Nov 07 16:12:59 2006
#   copyright (C) 2006 by Tim Niemueller, AllemaniACs RoboCup Team
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

SUBDIRS = core utils interface blackboard netcomm config aspect geometry

include $(BASEDIR)/etc/buildsys/config.mk
include $(BASEDIR)/etc/buildsys/rules.mk

# Explicit dependencies, this is needed to have make bail out if there is any
# error. This is also necessary for working parallel build (i.e. for dual core)
utils: core
interface: core
blackboard: core utils interface
netcomm: core utils
config: core netcomm utils
aspect: core utils blackboard netcomm config
geometry: core

