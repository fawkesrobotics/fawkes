#*****************************************************************************
#               Makefile Build System for Fawkes: kdl_parser
#                            -------------------
#   Created on Feb 14, 2014
#   Copyright (C) 2006-2014
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

HAVE_KDL=$(if $(shell $(PKGCONFIG) --exists 'orocos-kdl'; echo $${?/1/}),1,0)
ifeq ($(HAVE_KDL),1)
  CFLAGS_KDL = $(shell $(PKGCONFIG) --cflags 'orocos-kdl')
  LDFLAGS_KDL = $(shell $(PKGCONFIG) --libs 'orocos-kdl')
else
	# try again, maybe it's called orocos_kdl
  HAVE_KDL=$(if $(shell $(PKGCONFIG) --exists 'orocos_kdl'; echo $${?/1/}),1,0)
  ifeq ($(HAVE_KDL),1)
    CFLAGS_KDL = $(shell $(PKGCONFIG) --cflags 'orocos_kdl')
    LDFLAGS_KDL = $(shell $(PKGCONFIG) --libs 'orocos_kdl')
  endif
endif

HAVE_URDFDOMHEADERS=$(if $(shell $(PKGCONFIG) --exists 'urdfdom_headers'; echo $${?/1/}),1,0)
HAVE_URDFDOM=$(if $(shell $(PKGCONFIG) --exists 'urdfdom'; echo $${?/1/}),1,0)
HAVE_TINYXML=$(if $(wildcard $(SYSROOT)/usr/include/tinyxml.h),1,0)
HAVE_URDFDOM_TYPES_H=$(if $(shell $(PKGCONFIG) --atleast-version 0.4.0 'urdfdom_headers'; echo $${?/1/}),1,0)

ifeq ($(HAVE_KDL),1)
  ifeq ($(HAVE_URDFDOMHEADERS),1)
    ifeq ($(HAVE_URDFDOM),1)
      ifeq ($(HAVE_TINYXML),1)
        CFLAGS_KDLPARSER = $(CFLAGS_KDL) \
          $(shell $(PKGCONFIG) --cflags 'urdfdom_headers') \
          $(shell $(PKGCONFIG) --cflags 'urdfdom')
        LDFLAGS_KDLPARSER = $(LDFLAGS_KDL) \
          $(shell $(PKGCONFIG) --libs 'urdfdom_headers') \
          $(shell $(PKGCONFIG) --libs 'urdfdom') \
          -ltinyxml
        HAVE_KDLPARSER=1
        ifeq ($(HAVE_URDFDOM_TYPES_H),1)
          CFLAGS_KDLPARSER += -DHAVE_URDFDOM_TYPES_H
        endif
      endif
    endif
  endif
endif
