#*****************************************************************************
#            Makefile Build System for Fawkes: MongoDB Plugin
#                            -------------------
#   Created on Sun Dec 05 23:03:18 2010 (Steelers vs. Baltimore)
#   Copyright (C) 2006-2010 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

include $(BUILDSYSDIR)/boost.mk

MONGO_CXX_DRIVER_BOOST_LIBS = thread system

ifneq ($(wildcard /usr/include/mongo/client/dbclient.h /usr/local/include/mongo/client/dbclient.h),)
  ifneq ($(wildcard $(SYSROOT)/usr/include/mongo/version.h $(SYSROOT)/usr/local/include/mongo/version.h),)
    CFLAGS_MONGODB_VERSION_H += -DHAVE_MONGODB_VERSION_H
    ifeq ($(OS),FreeBSD)
      MONGO_CXX_DRIVER_BOOST_LIBS += regex
    endif
  else
    MONGO_CXX_DRIVER_BOOST_LIBS += regex filesystem
  endif

  ifeq ($(call boost-have-libs,$(MONGO_CXX_DRIVER_BOOST_LIBS)),1)
    HAVE_MONGODB = 1
    CFLAGS_MONGODB  = -DHAVE_MONGODB $(CFLAGS_CPP11) $(CFLAGS_MONGODB_VERSION_H)
    LDFLAGS_MONGODB = -lmongoclient -lm -lpthread \
		                  $(call boost-libs-ldflags,$(MONGO_CXX_DRIVER_BOOST_LIBS))

    ifeq ($(DISTRO),ubuntu)
      LDFLAGS_MONGODB += -lssl -lcrypto
    endif
    ifeq ($(OS),FreeBSD)
      CFLAGS_MONGODB  += -Wno-deprecated-declarations
      LDFLAGS_MONGODB += -lssl -lcrypto -lsasl2
    endif
  endif
endif

