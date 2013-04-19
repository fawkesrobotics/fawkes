#*****************************************************************************
#                 Makefile Build System for Fawkes: Boost bits
#                            -------------------
#   Created on Thu Apr 12 18:07:23 2012
#   Copyright (C) 2012 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

ifndef __buildsys_config_mk_
$(error config.mk must be included before boost.mk)
endif

ifndef __buildsys_boost_mk_
__buildsys_boost_mk_ := 1

BOOST_LIB_DIRS=/usr/lib64 /usr/lib /usr/lib32 \
	       /usr/local/lib64 /usr/local/lib
BOOST_INCLUDE_DIRS=
BOOST_USE_MULTITHREADED = 1
ifeq ($(BOOST_USE_MULTITHREADED),1)
  BOOST_LIBRARY_SUFFIX=-mt
endif


boost-have-libfile = $(if $(wildcard $(foreach l,$(BOOST_LIB_DIRS),$l/libboost_$1$(BOOST_LIBRARY_SUFFIX).$(SOEXT) )),1)
boost-have-lib     = $(if $(or $(call boost-have-libfile,$1),$(wildcard $(foreach i,$(BOOST_INCLUDE_DIRS) /usr/include /usr/local/include,$i/boost/$1.hpp))),1)
boost-lib-cflags   = $(addprefix -I,$(wildcard $(BOOST_INCLUDE_DIRS)))
boost-lib-ldflags  = $(if $(call boost-have-libfile,$1),-lboost_$1$(BOOST_LIBRARY_SUFFIX))

boost-have-libs    = $(if $(strip $(subst 1,,$(foreach l,$1,$(or $(call boost-have-lib,$l),0)))),,1)
boost-libs-cflags  = $(foreach l,$1,$(call boost-lib-cflags,$l))
boost-libs-ldflags = $(foreach l,$1,$(call boost-lib-ldflags,$l))

endif # __buildsys_boost_mk_
