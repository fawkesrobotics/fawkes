#*****************************************************************************
#           Makefile Build System for Fawkes: Webview Plugin Config
#                            -------------------
#   Created on Mon Oct 13 18:00:51 2008
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

ifneq ($(PKGCONFIG),)
  HAVE_LIBMICROHTTPD := $(if $(shell $(PKGCONFIG) --exists 'libmicrohttpd'; echo $${?/1/}),1,0)
  HAVE_RAPIDJSON := $(if $(shell $(PKGCONFIG) --exists 'RapidJSON'; echo $${?/1/}),1,0)
endif

ifeq ($(HAVE_LIBMICROHTTPD),1)
  CFLAGS_LIBMICROHTTPD  = -DHAVE_LIBMICROHTTPD $(shell $(PKGCONFIG) --cflags 'libmicrohttpd')
  LDFLAGS_LIBMICROHTTPD = $(shell $(PKGCONFIG) --libs 'libmicrohttpd')
  HAVE_WEBVIEW = 1
else
  ifneq ($(wildcard $(SYSROOT)/usr/include/microhttpd.h $(SYSROOT)/usr/local/include/microhttpd.h),)
    HAVE_LIBMICROHTTPD = 1
    CFLAGS_LIBMICROHTTPD = -DHAVE_LIBMICROHTTPD
    LDFLAGS_LIBMICROHTTPD = -lmicrohttpd
    HAVE_WEBVIEW = 1
  endif
endif

# RapidJSON is optional, but may be convenient
ifeq ($(HAVE_RAPIDJSON),1)
  CFLAGS_RAPIDJSON  = -DHAVE_RAPIDJSON $(shell $(PKGCONFIG) --cflags 'RapidJSON')
  LDFLAGS_RAPIDJSON = $(shell $(PKGCONFIG) --libs 'RapidJSON')
endif
