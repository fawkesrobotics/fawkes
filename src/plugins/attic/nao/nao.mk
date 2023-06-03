#*****************************************************************************
#   Makefile Build System for Fawkes: Config Settings specific to Nao
#                            -------------------
#   Created on Thu Aug 28 13:24:44 2008
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

NAOQIROOT=/opt/naoqi
#PYTHON_DIR?=/usr/include/python2.6/

ifneq ($(wildcard $(NAOQIROOT)/include/alcommon/almodule.h),)
  ifneq ($(TARGET_ARCH),x86_64)
    HAVE_NAOQI = 1
  else
    ifeq ($(DO_32BIT_BUILD),1)
      HAVE_NAOQI = 1
    else
      NAOQI_ERROR = "NaoQi only available on i386"
    endif
  endif
else
  NAOQI_ERROR = "NaoQi not found"
endif

ifeq ($(HAVE_NAOQI),1)
  CFLAGS_NAOQI  = -DHAVE_NAOQI -I$(NAOQIROOT)/include
  LDFLAGS_NAOQI = -L$(NAOQIROOT)/lib
endif

# Webots, we assume default location
# WEBOTS_HOME=/usr/local/webots

#NAOQI_MODS=naoqifawkes

