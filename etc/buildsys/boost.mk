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
	       /usr/local/lib64 /usr/local/lib \
	       /usr/lib/x86_64-linux-gnu/
BOOST_INCLUDE_DIRS=
BOOST_LIBRARY_SUFFIXES=-mt NOSUFFIX

boost-find-include = $(firstword $(wildcard $(foreach i,$(BOOST_INCLUDE_DIRS) /usr/include /usr/local/include,$i/boost/$1)))
boost-have-include = $(if $(call boost-find-include,$1),1)
boost-have-libfile = $(if $(wildcard $(foreach l,$(BOOST_LIB_DIRS),$(foreach s,$(BOOST_LIBRARY_SUFFIXES),$l/libboost_$1$(subst NOSUFFIX,,$s).$(SOEXT) ))),1)
boost-have-lib     = $(if $(or $(call boost-have-libfile,$1),$(call boost-have-include,$(1).hpp)),1)
boost-lib-cflags   = $(addprefix -I,$(wildcard $(BOOST_INCLUDE_DIRS))) $(BOOST_CFLAGS_$1)
boost-lib-ldflags  = $(addprefix -lboost_,$(foreach l,$(BOOST_LIB_DIRS),$(foreach s,$(BOOST_LIBRARY_SUFFIXES),$(if $(wildcard $l/libboost_$1$(subst NOSUFFIX,,$s).$(SOEXT)),$1$(subst NOSUFFIX,,$s) )))) $(BOOST_LDFLAGS_$1)

boost-have-libs    = $(if $(strip $(subst 1,,$(foreach l,$1,$(or $(call boost-have-lib,$l),0)))),,1)
boost-libs-cflags  = $(foreach l,$1,$(call boost-lib-cflags,$l))
boost-libs-ldflags = $(foreach l,$1,$(call boost-lib-ldflags,$l))

ifeq ($(call boost-have-include,version.hpp),1)
	HAVE_BOOST = 1
  BOOST_VERSION = $(shell LANG=C grep "define BOOST_VERSION " "$(call boost-find-include,version.hpp)" | awk '{ print $$3 }')
  BOOST_VERSION_MAJOR = $(shell echo $$(($(BOOST_VERSION) / 100000)))
  BOOST_VERSION_MINOR = $(shell echo $$(($(BOOST_VERSION) / 100 % 1000)))
  BOOST_VERSION_PATCH = $(shell echo $$(($(BOOST_VERSION) % 100)))
endif

boost-version-create = $(shell echo $$(($1 * 100000 + $2 * 100 + $3)))
boost-version-atleast = $(shell echo $$(($(BOOST_VERSION) >= $1 * 100000 + $2 * 100 + $3)))
boost-version-atmost = $(shell echo $$(($(BOOST_VERSION) <= $1 * 100000 + $2 * 100 + $3)))
boost-version-parse  = $(shell echo $$(($1 / 100000)).$$(($1 / 100 % 1000)).$$(($1 % 100)))

ifeq ($(CC),clang)
  ifeq ($(call boost-version-atmost,1,55,0),1)
    BOOST_CFLAGS_asio=-Wno-infinite-recursion -Wno-unused-local-typedef
  endif
endif

endif # __buildsys_boost_mk_
