#*****************************************************************************
#                Makefile Build System for Fawkes: Webview
#                            -------------------
#   Created on Fri Oct 03 14:33:30 2008 (19th German Unity Day)
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

HAVE_FCGI = $(if $(wildcard /usr/include/fcgi_stdio.h),1)
ifeq ($(HAVE_FCGI),1)
  LDFLAGS_FCGI = -lfcgi++
endif

#ifneq ($(PKGCONFIG),)
#  HAVE_CGICC = $(if $(shell $(PKGCONFIG) --exists 'cgicc'; echo $${?/1/}),1,0)
#endif
#ifeq ($(HAVE_CGICC),1)
#  CFLAGS_CGICC  = $(shell $(PKGCONFIG) --cflags 'cgicc')
#  LDFLAGS_CGICC = $(shell $(PKGCONFIG) --libs 'cgicc')
#endif

