#*****************************************************************************
#           Makefile Build System for Fawkes: OpenNI Plugin Config
#                            -------------------
#   Created on Sat Feb 26 15:53:37 2011
#   Copyright (C) 2011 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

ifneq ($(PKGCONFIG),)
  HAVE_OPENNI = $(if $(shell $(PKGCONFIG) --exists 'openni-dev'; echo $${?/1/}),1,0)
  ifeq ($(HAVE_OPENNI),1)
    CFLAGS_OPENNI  = $(shell $(PKGCONFIG) --cflags 'openni-dev')
    LDFLAGS_OPENNI = $(shell $(PKGCONFIG) --libs 'openni-dev') -lpthread
  endif
endif

ifneq ($(HAVE_OPENNI),1)
  ifneq ($(wildcard $(SYSROOT)/usr/include/ni/XnCppWrapper.h),)
    HAVE_OPENNI=1
    CFLAGS_OPENNI  = -I$(SYSROOT)/usr/include/ni
    LDFLAGS_OPENNI = -lOpenNI -lpthread
  endif
endif

ifeq ($(HAVE_OPENNI),1)
  CFLAGS_OPENNI += -DHAVE_OPENNI -Wno-unused-variable -Wno-reorder \
		  -fno-strict-aliasing -Wno-unknown-pragmas \
                  -Wno-return-type
  ifeq ($(CC),gcc)
    ifeq ($(call gcc_atleast_version,4,6),1)
      CFLAGS_OPENNI += -Wno-unused-but-set-variable
    endif
  endif
  ifeq ($(CC),clang)
    CFLAGS_OPENNI += -Wno-attributes
  endif
endif

