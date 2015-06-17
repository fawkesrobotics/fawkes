#*****************************************************************************
#                Makefile Build System for Fawkes: OpenPRS bits
#                            -------------------
#   Created on Fri Aug 22 13:23:51 2014
#   Copyright (C) 2014 by Tim Niemueller, AllemaniACs RoboCup Team
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
$(error config.mk must be included before openprs.mk)
endif

ifndef __buildsys_openprs_mk_
__buildsys_openprs_mk_ := 1

include $(BUILDSYSDIR)/boost.mk

OPENPRS_REQ_BOOST_LIBS = asio system
HAVE_OPENPRS_BOOST_LIBS = $(call boost-have-libs,$(OPENPRS_REQ_BOOST_LIBS))

# We use range-for loops
OPENPRS_GCC_MINV_MAJ = 4
OPENPRS_GCC_MINV_MIN = 6

OPENPRS_ERROR=

OPENPRS_MOD_DIR       = $(abspath $(LIBDIR)/openprs)
OPENPRS_EXEC_MOD_DIR  = $(abspath $(EXEC_LIBDIR)/openprs)
OPENPRS_EXEC_DATA_DIR = $(abspath $(BASEDIR)/src/plugins/openprs/data)

ifneq ($(PKGCONFIG),)
  HAVE_OPENPRS_MP = $(if $(shell $(PKGCONFIG) --exists 'mp-openprs'; echo $${?/1/}),1,0)
  ifeq ($(HAVE_OPENPRS_MP),1)
    CFLAGS_OPENPRS_MP  += -DHAVE_OPENPRS_MP $(shell $(PKGCONFIG) --cflags 'mp-openprs')
    LDFLAGS_OPENPRS_MP += $(shell $(PKGCONFIG) --libs 'mp-openprs')
  endif

  HAVE_OPENPRS_UTIL = $(if $(shell $(PKGCONFIG) --exists 'openprs-util'; echo $${?/1/}),1,0)
  ifeq ($(HAVE_OPENPRS_UTIL),1)
    CFLAGS_OPENPRS_UTIL  += -DHAVE_OPENPRS_UTIL $(shell $(PKGCONFIG) --cflags 'openprs-util')
    LDFLAGS_OPENPRS_UTIL += $(shell $(PKGCONFIG) --libs 'openprs-util')
  endif

  HAVE_OPENPRS_MOD = $(if $(shell $(PKGCONFIG) --exists 'openprs'; echo $${?/1/}),1,0)
  ifeq ($(HAVE_OPENPRS_MOD),1)
    CFLAGS_OPENPRS_MOD  += -DHAVE_OPENPRS_MOD $(shell $(PKGCONFIG) --cflags 'openprs')
    LDFLAGS_OPENPRS_MOD += $(shell $(PKGCONFIG) --libs 'openprs')
  endif

  ifeq ($(HAVE_OPENPRS_MP)$(HAVE_OPENPRS_UTIL)$(HAVE_OPENPRS_MOD)$(HAVE_OPENPRS_BOOST_LIBS),1111)
    HAVE_OPENPRS = 1

    CFLAGS_OPENPRS  = -DHAVE_OPENPRS -DOPENPRS_MOD_DIR=\"$(OPENPRS_EXEC_MOD_DIR)\" \
		      -DOPENPRS_DATA_DIR=\"$(OPENPRS_EXEC_DATA_DIR)\" \
		      $(CFLAGS_CPP11) $(call boost-libs-cflags,$(OPENPRS_REQ_BOOST_LIBS))
    LDFLAGS_OPENPRS = -lpthread $(call boost-libs-ldflags,$(OPENPRS_REQ_BOOST_LIBS))
  endif

  ifeq ($(HAVE_OPENPRS),1)
    ifeq ($(CC),gcc)
      ifneq ($(call gcc_atleast_version,$(OPENPRS_GCC_MINV_MAJ),$(OPENPRS_GCC_MINV_MIN)),1)
        HAVE_OPENPRS=
	OPENPRS_ERROR = GCC version too old, have $(GCC_VERSION), required $(OPENPRS_GCC_MINV_MAJ).$(OPENPRS_GCC_MINV_MIN)
      endif
    endif
  else
    ifneq ($(HAVE_OPENPRS_MP)$(HAVE_OPENPRS_UTIL)$(HAVE_OPENPRS_MOD),111)
      OPENPRS_ERROR = OpenPRS libs not found
    endif
    ifneq ($(HAVE_OPENPRS_BOOST_LIBS),1)
      OPENPRS_ERROR += Boost libraries missing
      $(foreach l,$(OPENPRS_REQ_BOOST_LIBS),$(if $(call boost-have-lib,$l),, warning_openprs_boost_$l))
    endif
  endif
else
  OPENPRS_ERROR = pkg-config not available
endif

ifeq ($(OBJSSUBMAKE),1)
  ifneq ($(OPENOPRS_WARN_TARGETS_BOOST),)
all: $(OPENPRS_WARN_TARGETS_BOOST)

.PHONY: $(OPENPRS_WARN_TARGETS_BOOST)
$(OPENPRS_WARN_TARGETS_BOOST): warning_openprs_boost_%:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)OpenPRS not available$(TNORMAL) (Boost library $* not found)"
  endif
endif

endif # __buildsys_openprs_mk_

