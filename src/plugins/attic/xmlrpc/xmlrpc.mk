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

ifeq ($(HAVE_LIBXMLRPCPP),0)
  HAVE_XMLRPC_C_CONFIG := $(if $(shell which xmlrpc-c-config; echo $${?/1/}),1,0)
  HAVE_LIBXMLRPCPP := 1
endif

ifeq ($(HAVE_LIBXMLRPCPP),1)
  HAVE_XMLRPC = 1
  ifeq ($(HAVE_XMLRPC_C_CONFIG),1)
    CFLAGS_LIBXMLRPCPP  = $(shell xmlrpc-c-config c++2 --cflags)
    LDFLAGS_LIBXMLRPCPP = $(shell xmlrpc-c-config c++2 --libs)
  else
    CFLAGS_LIBXMLRPCPP  = $(shell $(PKGCONFIG) --cflags $(LIBXMLPP_PACKAGES))
    LDFLAGS_LIBXMLRPCPP = $(shell $(PKGCONFIG) --libs $(LIBXMLPP_PACKAGES))
  endif
else
  # FreeBSD
  ifneq ($(wildcard $(SYSROOT)/usr/local/include/xmlrpc-c/registry.hpp),)
    HAVE_LIBXMLRPCPP = 1
    HAVE_XMLRPC = 1
    LDFLAGS_LIBXMLRPCPP = -lxmlrpc_server++ -lxmlrpc++
  endif
  # Mac OS X
  ifneq ($(wildcard $(SYSROOT)/opt/local/include/xmlrpc-c/registry.hpp),)
    HAVE_LIBXMLRPCPP = 1
    HAVE_XMLRPC = 1
    LDFLAGS_LIBXMLRPCPP = -lxmlrpc_server -lxmlrpc_server++ -lxmlrpc++ -lxmlrpc_util -lxmlrpc_abyss \
			  -lxmlrpc_server_abyss++ -lxmlrpc_cpp -lxmlrpc \
			  -lxmlrpc_server_abyss -lxmlrpc_xmlparse -lxmlrpc_xmltok
  endif
  # Ubuntu
  ifneq ($(wildcard $(SYSROOT)/usr/include/xmlrpc-c/registry.hpp),)
    HAVE_LIBXMLRPCPP = 1
    HAVE_XMLRPC = 1
    LDFLAGS_LIBXMLRPCPP = -lxmlrpc_server++ -lxmlrpc++ -lxmlrpc -lxmlrpc_xmlparse \
			  -lxmlrpc_xmltok -lxmlrpc_server
  endif
endif

