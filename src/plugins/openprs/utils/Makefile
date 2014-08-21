#*****************************************************************************
#             Makefile Build System for Fawkes: OpenPRS Utils
#                            -------------------
#   Created on Wed Aug 20 18:19:01 2014
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

BASEDIR = ../../../..

include $(BASEDIR)/etc/buildsys/config.mk
include $(LIBSRCDIR)/utils/utils.mk
include $(BUILDSYSDIR)/boost.mk

REQ_BOOST_LIBS = asio system
HAVE_BOOST_LIBS = $(call boost-have-libs,$(REQ_BOOST_LIBS))

HAVE_OPENPRS_MP = $(if $(shell $(PKGCONFIG) --exists 'mp-openprs'; echo $${?/1/}),1,0)
ifeq ($(HAVE_OPENPRS_MP),1)
  CFLAGS_OPENPRS_MP  += -DHAVE_OPENPRS_MP $(shell $(PKGCONFIG) --cflags 'mp-openprs')
  LDFLAGS_OPENPRS_MP += $(shell $(PKGCONFIG) --libs 'mp-openprs')
endif


CFLAGS  += $(CFLAGS_CPP11) $(CFLAGS_OPENPRS_MP)
LDFLAGS += $(LDFLAGS_OPENPRS_MP)

LIBS_libfawkesopenprsutils = stdc++ pthread fawkescore fawkesutils
OBJS_libfawkesopenprsutils = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))

OBJS_all    = $(OBJS_libfawkesopenprsutils)

ifeq ($(CC),gcc)
  ifneq ($(call gcc_atleast_version,4,6),1)
    GCC_TOO_OLD=1
  endif
endif

ifneq ($(GCC_TOO_OLD),1)
  ifeq ($(HAVE_OPENPRS_MP)$(HAVE_BOOST_LIBS),11)
    CFLAGS  += $(call boost-libs-cflags,$(REQ_BOOST_LIBS)) 
    LDFLAGS += $(call boost-libs-ldflags,$(REQ_BOOST_LIBS))

    LIBS_all = $(LIBDIR)/libfawkesopenprsutils.so
  else
    ifneq ($(HAVE_OPENPRS_MP),1)
      WARN_TARGETS += warning_openprs_mp
    endif
    ifneq ($(HAVE_BOOST_LIBS),1)
      WARN_TARGETS_BOOST = $(foreach l,$(REQ_BOOST_LIBS),$(if $(call boost-have-lib,$l),, warning_boost_$l))
    endif
  endif
else
  WARN_TARGETS += warning_old_gcc
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS) $(WARN_TARGETS_BOOST)

.PHONY: warning_openprs warning_old_gcc $(WARN_TARGETS_BOOST)
warning_openprs:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting OpenPRS utils$(TNORMAL) (OpenPRS MP libs not found)"
warning_old_gcc:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting OpenPRS utils$(TNORMAL) (GCC too old, have $(GCC_VERSION), required 4.6)"
$(WARN_TARGETS_BOOST): warning_boost_%:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting OpenPRS utils$(TNORMAL) (Boost library $* not found)"
endif

include $(BUILDSYSDIR)/base.mk

