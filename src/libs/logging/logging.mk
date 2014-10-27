#*****************************************************************************
#           Makefile Build System for Fawkes: Logging Utils
#                            -------------------
#   Created on Mon Oct 27 16:11:45 2014
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

LOGGING_REQ_BOOST_LIBS = asio system
LOGGING_HAVE_BOOST_LIBS = $(call boost-have-libs,$(LOGGING_REQ_BOOST_LIBS))

ifeq ($(CC),gcc)
  ifeq ($(call gcc_atleast_version,4,6),1)
    LOGGING_FD_REDIRECT_GCC_OK=1
  endif
endif
ifeq ($(CC),clang)
  LOGGING_FD_REDIRECT_GCC_OK=1
endif

ifeq ($(LOGGING_FD_REDIRECT_GCC_OK)$(LOGGING_HAVE_BOOST_LIBS),11)
  HAVE_LOGGING_FD_REDIRECT=1
endif
