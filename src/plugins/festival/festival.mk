#*****************************************************************************
#           Makefile Build System for Fawkes: Flite Plugin Config
#                            -------------------
#   Created on Tue Oct 28 14:50:59 2008
#   Copyright (C) 2008 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

include $(BASEDIR)/etc/buildsys/ext/gmsl

ifneq ($(wildcard $(SYSROOT)/usr/include/festival/festival.h),)
  FESTIVAL_BIN=$(shell which festival)
  ifneq ($(FESTIVAL_BIN),)
    FESTIVAL_VERSION=$(strip $(shell $(FESTIVAL_BIN) --version | awk -F: '{ print $$3 }'))
    FESTIVAL_VERSION_ELEMENTS=$(call split,.,$(FESTIVAL_VERSION))
    FESTIVAL_VERSION_MINOR=$(word 2,$(FESTIVAL_VERSION_ELEMENTS))
    FESTIVAL_VERSION_MAJOR=$(firstword $(FESTIVAL_VERSION_ELEMENTS))
    HAVE_FESTIVAL=$(if $(patsubst 0,,$(shell expr "(" $(FESTIVAL_VERSION_MAJOR) ">" 1 ")" "|" "(" $(FESTIVAL_VERSION_MINOR) ">=" 96 ")")),1,0)
  endif
  ifeq ($(HAVE_FESTIVAL),1)
    CFLAGS_FESTIVAL = -I/usr/include/speech_tools
    LIBS_FESTIVAL   = Festival estbase estools eststring
  else
    ifneq ($(FESTIVAL_BIN),)
      FESTIVAL_ERROR=Festival version too old ($(FESTIVAL_VERSION) < 1.96)
    else
      FESTIVAL_ERROR=Festival binary not installed, cannot determine version
    endif
  endif
else
  FESTIVAL_ERROR=festival[-devel] not installed
endif

