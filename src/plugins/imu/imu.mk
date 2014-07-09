#*****************************************************************************
#            Makefile Build System for Fawkes: IMU Plugin
#                            -------------------
#   Created on Sun Jun 22 19:29:54 2014
#   Copyright (C) 2006-2014 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BUILDCONFDIR)/tf/tf.mk

CRUIZCORE_REQ_BOOST_LIBS = thread asio system
CRUIZCORE_HAVE_BOOST_LIBS = $(call boost-have-libs,$(CRUIZCORE_REQ_BOOST_LIBS))

ifeq ($(CRUIZCORE_HAVE_BOOST_LIBS)$(HAVE_TF),11)
  HAVE_CRUIZCORE    = 1
  CFLAGS_CRUIZCORE  = -DHAVE_CRUIZCORE $(CFLAGS_TF) \
		      $(call boost-libs-cflags,$(CRUIZCORE_REQ_BOOST_LIBS))
  LDFLAGS_CRUIZCORE = $(LDFLAGS_TF) $(call boost-libs-ldflags,$(CRUIZCORE_REQ_BOOST_LIBS))
endif
