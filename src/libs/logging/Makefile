#*****************************************************************************
#           Makefile Build System for Fawkes: Logging Utils
#                            -------------------
#   Created on Sat May 28 00:16:57 2011
#   Copyright (C) 2006-2017 by Tim Niemueller, RWTH Aachen University
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
include $(BUILDSYSDIR)/boost.mk

LIBS_all  = $(LIBDIR)/libfawkeslogging.so

ifeq ($(HAVE_TOLUA),1)
  LIBS_all += $(LUALIBDIR)/fawkeslogging.so
  TARGETS_all += $(SRCDIR)/fawkeslogging_tolua.cpp
endif

LIBS_libfawkeslogging = stdc++ pthread fawkescore
OBJS_libfawkeslogging =$(filter-out %_tolua.o,$(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp))))))
HDRS_libfawkeslogging = $(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.h))

CFLAGS_fawkeslogging_tolua = -Wno-unused-function $(CFLAGS_LUA)
TOLUA_fawkeslogging = $(wildcard $(SRCDIR)/*.tolua)
LDFLAGS_lua_fawkeslogging= $(LDFLAGS_LUA)
LIBS_lua_fawkeslogging = fawkescore fawkeslogging $(TOLUA_LIBS)
OBJS_lua_fawkeslogging = fawkeslogging_tolua.o

OBJS_all = $(OBJS_libfawkeslogging) $(OBJS_lua_fawkeslogging)

ifneq ($(HAVE_LOGGING_FD_REDIRECT),1)
  OBJS_libfawkeslogging := $(filter-out fd_redirect.o,$(OBJS_libfawkeslogging))

  ifneq ($(LOGGING_FD_REDIRECT_GCC_OK),1)
    WARN_TARGETS += warning_fd_redirect_gcc
  endif
  ifneq ($(LOGGING_HAVE_BOOST_LIBS),1)
    WARN_TARGETS_BOOST = $(foreach l,$(LOGGING_REQ_BOOST_LIBS),$(if $(call boost-have-lib,$l),, warning_boost_$l))
  endif
else
  CFLAGS_fd_redirect += $(CFLAGS_CPP11) $(call boost-libs-cflags,$(LOGGING_REQ_BOOST_LIBS))
  LDFLAGS_libfawkeslogging += $(call boost-libs-ldflags,$(LOGGING_REQ_BOOST_LIBS))
endif

$(LUALIBDIR)/fawkeslogging.so: | $(LIBDIR)/libfawkeslogging.so

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)  $(WARN_TARGETS_BOOST)

.PHONY: warning_fd_redirect_gcc $(WARN_TARGETS_BOOST)
warning_fd_redirect_gcc:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting file descriptor redirect support$(TNORMAL) (GCC too old, have $(GCC_VERSION), required 4.6)"
$(WARN_TARGETS_BOOST): warning_boost_%:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting file descriptor redirect support$(TNORMAL) (Boost library $* not found)"
endif


include $(BUILDSYSDIR)/base.mk

