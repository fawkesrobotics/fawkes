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

ifneq ($(wildcard $(SYSROOT)/usr/include/flite/flite.h $(SYSROOT)/usr/local/include/flite/flite.h),)
  HAVE_FLITE=1
endif

ifeq ($(HAVE_FLITE),1)
  ifeq ($(DISTRO),arch)
    CFLAGS_FLITE += -lflite_cmu_us_kal -lflite_usenglish -lflite_cmulex -lflite
    LDFLAGS_FLITE += -lflite_cmu_us_kal -lflite_usenglish -lflite_cmulex -lflite
  endif
endif

ifneq ($(PKGCONFIG),)
  HAVE_ALSA = $(if $(shell $(PKGCONFIG) --exists 'alsa'; echo $${?/1/}),1,0)
  CFLAGS_ALSA  = $(shell $(PKGCONFIG) --cflags 'alsa')
  LDFLAGS_ALSA = $(shell $(PKGCONFIG) --libs 'alsa')
endif

