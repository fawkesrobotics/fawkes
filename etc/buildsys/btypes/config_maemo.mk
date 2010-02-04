#*****************************************************************************
#   Makefile Build System for Fawkes: Config Settings specific to Fawkes
#                            -------------------
#   Created on Thu Oct 16 20:00:11 2008
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

include $(BASEDIR)/etc/buildsys/btypes/config_fawkes.mk
include $(BASEDIR)/etc/buildsys/ext/gmsl

MAEMO_VERSION=$(shell awk '{ print $$1 }' /etc/maemo_version)
MAEMO_VERSION_SPLITTED=$(call split,.,$(MAEMO_VERSION))
MAEMO_VERSION_MAJOR=$(word 1,$(MAEMO_VERSION_SPLITTED))
MAEMO_VERSION_MINOR=$(word 2,$(MAEMO_VERSION_SPLITTED))
MAEMO_VERSION_MICRO=$(word 3,$(MAEMO_VERSION_SPLITTED))
ifeq ($(MAEMO_VERSION_MICRO),)
  MAEMO_VERSION_MICRO=0
endif

EXEC_BASEDIR=/home/user/fawkes
GCC_USE_OPENMP=0

CFLAGS += -DMAEMO_VERSION_MAJOR=$(MAEMO_VERSION_MAJOR) \
          -DMAEMO_VERSION_MINOR=$(MAEMO_VERSION_MINOR) \
          -DMAEMO_VERSION_MICRO=$(MAEMO_VERSION_MICRO)

