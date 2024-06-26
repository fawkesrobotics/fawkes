#*****************************************************************************
#              Makefile Build System for Fawkes: Utility Library
#                            -------------------
#   Created on Sun Sep 03 14:14:14 2006
#   Copyright (C) 2006-2008 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

BASEDIR = ../../..
include $(BASEDIR)/etc/buildsys/config.mk
include $(BUILDSYSDIR)/lua.mk
include $(BASEDIR)/etc/buildsys/boost.mk

FILTER_OUT=%_tolua.o tests/%

REQ_BOOST_LIBS = asio system
HAVE_BOOST_LIBS = $(call boost-have-libs,$(REQ_BOOST_LIBS))

ifneq ($(wildcard $(SYSROOT)/usr/include/magic.h $(SYSROOT)/opt/local/include/magic.h),)
  HAVE_LIBMAGIC = 1
  LIBS_LIBMAGIC = magic
  CFLAGS_LIBMAGIC = -DHAVE_LIBMAGIC
else
  WARN_TARGETS += warning_libmagic
endif

LIBS_libfawkesutils = stdc++ m rt fawkescore fawkeslogging $(LIBS_LIBMAGIC) $(if $(filter-out Darwin,$(OS)),rt) $(if $(filter Linux,$(OS)),dl pthread) uuid
OBJS_libfawkesutils =	$(filter-out $(FILTER_OUT),$(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp $(SRCDIR)/*/*.cpp $(SRCDIR)/*/*/*.cpp))))))
HDRS_libfawkesutils = $(subst $(SRCDIR)/,,$(filter-out $(patsubst %.o,%.h,$(FILTER_OUT)),$(wildcard $(SRCDIR)/*.h $(SRCDIR)/*/*.h)))


CFLAGS_fawkesutils_tolua = -Wno-unused-function $(CFLAGS_LUA)
TOLUA_fawkesutils = $(wildcard $(SRCDIR)/*.tolua $(SRCDIR)/*/*.tolua $(SRCDIR)/*/*/*.tolua)
LDFLAGS_lua_fawkesutils = $(LDFLAGS_LUA)
LIBS_lua_fawkesutils = fawkescore fawkesutils $(TOLUA_LIBS)
OBJS_lua_fawkesutils = fawkesutils_tolua.o

CFLAGS  += $(CFLAGS_LIBMAGIC)
LDFLAGS += $(LDFLAGS_LIBMAGIC)

HAVE_LIBDAEMON = $(if $(shell $(PKGCONFIG) --exists 'libdaemon'; echo $${?/1/}),1,0)
ifeq ($(HAVE_LIBDAEMON),1)
  CFLAGS  +=  -DHAVE_LIBDAEMON $(shell $(PKGCONFIG) --cflags 'libdaemon')
  LDFLAGS += $(shell $(PKGCONFIG) --libs 'libdaemon')
else
  WARN_TARGETS += warning_libdaemon
endif

OBJS_all = $(OBJS_libfawkesutils) $(OBJS_lua_fawkesutils)
LIBS_all = $(LIBDIR)/libfawkesutils.so \
           $(LUALIBDIR)/fawkesutils.so

ifeq ($(HAVE_CPP11)$(HAVE_BOOST_LIBS),11)
  CFLAGS += $(CFLAGS_CPP11) $(call boost-libs-cflags,$(REQ_BOOST_LIBS))
  LIBS_build += $(LUALIBDIR)/fawkesutils.so
  LDFLAGS += $(call boost-libs-ldflags,$(REQ_BOOST_LIBS))

  ifeq ($(HAVE_TOLUA),1)
    LIBS_build  += $(LIBDIR)/libfawkesutils.so
    TARGETS_all += $(SRCDIR)/fawkesutils_tolua.cpp
  endif
else
  LIBS_skipped = fawkesutils
  ifneq ($(HAVE_CPP11),1)
    WARN_TARGETS += warning_cpp11
		LIBS_skipped += ",no C++11"
  endif
  ifneq ($(HAVE_TOLUA),1)
    WARN_TARGETS += warning_tolua_wrapper
		LIBS_skipped += ",no tolua++"
  endif
  ifneq ($(HAVE_BOOST_LIBS),1)
    WARN_TARGETS_BOOST = $(foreach l,$(REQ_BOOST_LIBS),$(if $(call boost-have-lib,$l),, warning_boost_$l))
		LIBS_skipped += ",no Boost"
  endif
endif

ifneq ($(HAVE_INOTIFY),1)
  WARN_TARGETS += warning_inotify
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS) $(WARN_TARGETS_BOOST)
.PHONY: warning_libmagic warning_inotify
warning_libmagic:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)Restricted file type detection support$(TNORMAL) (file[-devel] (libmagic) not installed)"
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)File monitoring utils cannot be built$(TNORMAL) (inotify headers not found)"
warning_cpp11:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting utils$(TNORMAL) " \
		"(CPP11 not available)"
$(WARN_TARGETS_BOOST): warning_boost_%:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build utils library$(TNORMAL) (Boost library $* not found)"endif
warning_libdaemon:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Sub-process cannot close all file handles$(TNORMAL) (libdaemon not found)"

endif

$(LUALIBDIR)/fawkesutils.$(SOEXT): | $(LIBDIR)/libfawkesutils.$(SOEXT)

include $(BUILDSYSDIR)/base.mk
