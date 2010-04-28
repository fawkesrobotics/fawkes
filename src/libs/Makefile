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

BASEDIR = ../..

SUBDIRS = core utils interface netcomm blackboard config lua plugin aspect \
	  geometry webview worldinfo_utils gui_utils \
	  $(subst $(CURDIR)/,,$(wildcard interfaces))

include $(BASEDIR)/etc/buildsys/config.mk
include $(BUILDSYSDIR)/rules.mk

# Explicit dependencies, this is needed to have make bail out if there is any
# error. This is also necessary for working parallel build (i.e. for dual core)
utils: core 
interface: core utils
blackboard: core utils interface netcomm
netcomm: core utils
config: core netcomm utils
plugin: core netcomm utils config
aspect: core utils blackboard netcomm config plugin
geometry: core
worldinfo_utils: core netcomm geometry
lua: core utils blackboard config
interfaces: core interface
gui_utils: core netcomm plugin
naoutils: core utils
webview: core utils

