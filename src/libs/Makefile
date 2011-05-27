#*****************************************************************************
#                  Makefile Build System for Fawkes: Libraries
#                            -------------------
#   Created on Tue Nov 07 16:12:59 2006
#   copyright (C) 2006-2011 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BASEDIR)/etc/buildsys/config.mk
include $(BUILDSYSDIR)/fvconf.mk

SUBDIRS = core utils interface netcomm blackboard config logging lua plugin \
	  aspect geometry webview worldinfo_utils gui_utils baseapp \
	  fvutils fvcams fvmodels fvfilters fvclassifiers fvstereo fvwidgets \
	  $(subst $(CURDIR)/,,$(wildcard interfaces))

ifeq ($(HAVE_SIFT),1)
  SUBDIRS += extlib/sift
  CLASSIFIERS_DEPS += extlib/sift
endif
ifeq ($(HAVE_SURF),1)
  SUBDIRS += extlib/surf
  CLASSIFIERS_DEPS += extlib/surf
endif
ifeq ($(HAVE_SIFTPP),1)
  SUBDIRS += extlib/siftpp
  CLASSIFIERS_DEPS += extlib/siftpp
endif


# Explicit dependencies, this is needed to have make bail out if there is any
# error. This is also necessary for working parallel build (i.e. for dual core)
utils: core
interface: core utils
blackboard: core utils interface netcomm logging
netcomm: core utils
config: core netcomm utils logging
plugin: core netcomm utils config logging
aspect: core utils blackboard netcomm config plugin webview logging
geometry: core
worldinfo_utils: core netcomm geometry
lua: core utils blackboard config logging
interfaces: core interface
gui_utils: core netcomm plugin logging
logging: core utils netcomm
naoutils: core utils
webview: core utils logging
baseapp: core utils aspect config netcomm blackboard plugin logging
fvutils: core utils netcomm logging
fvcams fvmodels fvfilters fvclassifiers fvstereo fvwidgets: core utils fvutils logging
fvfilters: fvmodels
fvmodels: fvclassifiers geometry
fvstereo: fvcams
ifneq ($(CLASSIFIERS_DEPS),)
fvclassifiers: $(CLASSIFIERS_DEPS)
endif

include $(BUILDSYSDIR)/rules.mk
include $(LIBSRCDIR)/extlibs.mk

