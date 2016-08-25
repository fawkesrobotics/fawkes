#*****************************************************************************
#            Makefile Build System for Fawkes: Robotino Plugin
#                            -------------------
#   Created on Fri Jan 07 23:44:11 2011
#   Copyright (C) 2006-2011 by Tim Niemueller, AllemaniACs RoboCup Team
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

OPENROBOTINOAPI_DIR=/usr/local/OpenRobotinoAPI
OPENROBOTINOAPI2_DIR=/usr/local/robotino

ifneq ($(wildcard $(OPENROBOTINOAPI2_DIR)/api2/include/rec/robotino/api2/rec_robotino_api2_version.h),)
HAVE_OPENROBOTINO=1
OPENROBOTINO_API_VERSION = 2
CFLAGS_OPENROBOTINO  = -DHAVE_OPENROBOTINO_API_2 \
		   -I$(OPENROBOTINOAPI2_DIR)/api2/include -I$(OPENROBOTINOAPI2_DIR)/common/include
LDFLAGS_OPENROBOTINO = -L$(OPENROBOTINOAPI2_DIR)/api2/lib -L$(OPENROBOTINOAPI2_DIR)/common/lib \
		   -lrec_robotino_api2
else
  ifneq ($(wildcard $(OPENROBOTINOAPI_DIR)/1/include/robotinoapi_version.h),)
	HAVE_OPENROBOTINO=1
	OPENROBOTINO_API_VERSION = 1
	CFLAGS_OPENROBOTINO  = -DHAVE_OPENROBOTINO_API_1 \
			   -Wno-reorder -Wno-unused-function -Wno-delete-non-virtual-dtor \
			   -I$(OPENROBOTINOAPI_DIR)/1/include -I$(OPENROBOTINOAPI_DIR)/share/include
	LDFLAGS_OPENROBOTINO = -L$(OPENROBOTINOAPI_DIR)/1/lib/linux -L$(OPENROBOTINOAPI_DIR)/share/lib \
			   -lrec_robotino_com -lrec_core_lt \
			   -lrec_iocontrol_robotstate -lrec_iocontrol_remotestate
  endif
endif

HAVE_LIBUDEV=$(if $(shell $(PKGCONFIG) --exists 'libudev'; echo $${?/1/}),1,0)
ifeq ($(HAVE_LIBUDEV),1)
  CFLAGS_LIBUDEV  = -DHAVE_LIBUDEV $(shell $(PKGCONFIG) --cflags 'libudev')
  LDFLAGS_LIBUDEV = $(shell $(PKGCONFIG) --libs 'libudev')
endif

HAVE_LIBUSB=$(if $(shell $(PKGCONFIG) --exists 'libusb-1.0'; echo $${?/1/}),1,0)
ifeq ($(HAVE_LIBUSB),1)
  CFLAGS_LIBUSB  = -DHAVE_LIBUSB $(shell $(PKGCONFIG) --cflags 'libusb-1.0')
  LDFLAGS_LIBUSB = $(shell $(PKGCONFIG) --libs 'libusb-1.0')
endif

ROBOTINO_DIRECT_REQ_BOOST_LIBS = thread asio system filesystem
HAVE_ROBOTINO_DIRECT_BOOST_LIBS = $(call boost-have-libs,$(ROBOTINO_DIRECT_REQ_BOOST_LIBS))
ifeq ($(HAVE_ROBOTINO_DIRECT_BOOST_LIBS),1)
  CFLAGS_ROBOTINO_DIRECT_BOOST  = $(call boost-libs-cflags,$(ROBOTINO_DIRECT_REQ_BOOST_LIBS))
  LDFLAGS_ROBOTINO_DIRECT_BOOST = $(call boost-libs-ldflags,$(ROBOTINO_DIRECT_REQ_BOOST_LIBS)) -lpthread
endif

