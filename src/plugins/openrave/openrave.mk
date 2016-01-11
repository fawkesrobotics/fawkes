#*****************************************************************************
#              Makefile Build System for Fawkes: OpenRAVE Plugin
#                            -------------------
#   Created on Fri Feb 25 15:08:00 2011
#   Copyright (C) 2011 by Bahram Maleki-Fard, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

include $(BUILDSYSDIR)/boost.mk

OPENRAVE_MIN_VERSION=0.7.0

#Check for OpenRAVE
ifneq ($(PKGCONFIG),)
  HAVE_OPENRAVE = $(if $(shell $(PKGCONFIG) --atleast-version $(OPENRAVE_MIN_VERSION) 'openrave-core'; echo $${?/1/}),1,0)
  ifneq ($(HAVE_OPENRAVE),1)
    # Give it another shot, name might contain version
    VERSION_SPLITTED = $(call split,.,$(OPENRAVE_MIN_VERSION))
    VERSION_MAJOR    = $(word 1,$(VERSION_SPLITTED))
    VERSION_MINOR    = $(word 2,$(VERSION_SPLITTED))
    OPENRAVE_VERSION_INFIX = $(VERSION_MAJOR).$(VERSION_MINOR)
    _OPENRAVE_ALTERNATE_NAME = $(shell $(PKGCONFIG) --list-all | grep 'openrave$(OPENRAVE_VERSION_INFIX)-core' | awk '{ print $$1 }')
    ifneq ($(_OPENRAVE_ALTERNATE_NAME),)
      HAVE_OPENRAVE = $(if $(shell $(PKGCONFIG) --atleast-version $(OPENRAVE_MIN_VERSION) '$(_OPENRAVE_ALTERNATE_NAME)'; echo $${?/1/}),1,0)
    endif
  endif

  HAVE_BOOST_THREAD = $(call boost-have-lib,thread)
  ifneq ($(HAVE_BOOST_THREAD),1)
    HAVE_OPENRAVE = 0
  endif

  ifeq ($(HAVE_OPENRAVE),1)
    HAVE_PYTHON := $(if $(shell $(PKGCONFIG) --exists 'python'; echo $${?/1/}),1,0)
  endif
endif

ifeq ($(HAVE_OPENRAVE),1)
  CFLAGS_OPENRAVE  = -DHAVE_OPENRAVE \
                     $(shell $(PKGCONFIG) --cflags 'openrave$(OPENRAVE_VERSION_INFIX)-core') \
                     $(shell $(PKGCONFIG) --cflags 'openrave$(OPENRAVE_VERSION_INFIX)') \
                     $(call boost-lib-cflags,thread) \
		     -Wno-deprecated-declarations
  LDFLAGS_OPENRAVE = $(shell $(PKGCONFIG) --libs 'openrave$(OPENRAVE_VERSION_INFIX)-core') \
                     $(shell $(PKGCONFIG) --libs 'openrave$(OPENRAVE_VERSION_INFIX)') \
                     $(call boost-lib-ldflags,thread)
endif

ifeq ($(HAVE_PYTHON),1)
  CFLAGS_PYTHON    = -DHAVE_PYTHON $(shell $(PKGCONFIG) --cflags 'python')
  LDFLAGS_PYTHON   = $(shell $(PKGCONFIG) --libs 'python')
endif
