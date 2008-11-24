#*****************************************************************************
#                      Makefile Build System for Fawkes
#                            -------------------
#   Created on Wed Sep 06 02:39:46 2006
#   Copyright (C) 2006-2008 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

BASEDIR = ..

SUBDIRS = $(subst $(CURDIR)/,,$(realpath libs interfaces mainapp firevision plugins tools))

include $(BASEDIR)/etc/buildsys/config.mk
include $(BASEDIR)/etc/buildsys/rules.mk

# Explicit dependencies, this is needed to have make bail out if there is any
# error. This is also necessary for working parallel build (i.e. for dual core)
interfaces mainapp firevision plugins tools: libs
mainapp plugins firevision tools: interfaces
tools: mainapp
plugins: firevision

