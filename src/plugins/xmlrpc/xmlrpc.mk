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

LIBXMLPP_PACKAGES=xmlrpc_server++ xmlrpc++

ifneq ($(PKGCONFIG),)
  HAVE_LIBXMLRPCPP := $(if $(shell $(PKGCONFIG) --exists $(LIBXMLPP_PACKAGES); echo $${?/1/}),1,0)
endif

ifeq ($(HAVE_LIBXMLRPCPP),1)
  HAVE_XMLRPC = 1
  CFLAGS_LIBXMLRPCPP  = $(shell $(PKGCONFIG) --cflags $(LIBXMLPP_PACKAGES))
  LDFLAGS_LIBXMLRPCPP = $(shell $(PKGCONFIG) --libs $(LIBXMLPP_PACKAGES))
else
  # Might be FreeBSD
  ifneq ($(wildcard $(SYSROOT)/usr/local/include/xmlrpc-c/registry.hpp),)
    HAVE_LIBXMLRPCPP = 1
    HAVE_XMLRPC = 1
    LDFLAGS_LIBXMLRPCPP = -lxmlrpc_server++ -lxmlrpc++
  endif
endif

