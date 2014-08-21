#*****************************************************************************
#               Makefile Build System for Fawkes: OpenPRS Plugin
#                            -------------------
#   Created on Thu Aug 14 15:53:07 2014
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

BASEDIR = ../../..
include $(BASEDIR)/etc/buildsys/config.mk
# include $(BUILDSYSDIR)/openprs.mk
include $(BUILDSYSDIR)/boost.mk

REQ_BOOST_LIBS = asio system thread
HAVE_BOOST_LIBS = $(call boost-have-libs,$(REQ_BOOST_LIBS))

LIBS_openprs = pthread fawkescore fawkesutils fawkesaspects fawkesblackboard \
               fawkesinterface fawkesopenprsaspect fawkesopenprsutils
OBJS_openprs = openprs_plugin.o openprs_thread.o

OBJS_all = $(OBJS_openprs)

ifeq ($(CC),gcc)
  ifneq ($(call gcc_atleast_version,4,6),1)
    GCC_TOO_OLD=1
  endif
endif

HAVE_LIBDAEMON = $(if $(shell $(PKGCONFIG) --exists 'libdaemon'; echo $${?/1/}),1,0)
ifeq ($(HAVE_LIBDAEMON),1)
  CFLAGS_LIBDAEMON  += -DHAVE_LIBDAEMON $(shell $(PKGCONFIG) --cflags 'libdaemon')
  LDFLAGS_LIBDAEMON += $(shell $(PKGCONFIG) --libs 'libdaemon')
else
  WARN_TARGETS += warning_libdaemon
endif

ifneq ($(GCC_TOO_OLD),1)
  ifeq ($(HAVE_LIBDAEMON)$(HAVE_BOOST_LIBS),11)
    PRESUBDIRS += utils aspect
    SUBDIRS    += example

    CFLAGS  += $(CFLAGS_CPP11) \
	       $(CFLAGS_OPENPRS)  $(CFLAGS_LIBDAEMON) \
	       $(call boost-libs-cflags,$(REQ_BOOST_LIBS)) 
    LDFLAGS += $(LDFLAGS_OPENPRS) $(LDFLAGS_LIBDAEMON) \
	       $(call boost-libs-ldflags,$(REQ_BOOST_LIBS))

    PLUGINS_all = $(PLUGINDIR)/openprs.so
  else
    ifneq ($(HAVE_LIBDAEMON),1)
      WARN_TARGETS += warning_libdaemon
    endif
    ifneq ($(HAVE_BOOST_LIBS),1)
      WARN_TARGETS_BOOST = $(foreach l,$(REQ_BOOST_LIBS),$(if $(call boost-have-lib,$l),, warning_boost_$l))
    endif
  endif
else
  WARN_TARGETS += warning_old_gcc
endif

aspect: utils
example: aspect utils

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)  $(WARN_TARGETS_BOOST)

.PHONY: warning_openprs warning_old_gcc $(WARN_TARGETS_BOOST)
warning_clips:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting OpenPRS Plugins$(TNORMAL) ($(OPENPRS_ERROR))"
warning_old_gcc:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting OpenPRS Plugins$(TNORMAL) (GCC too old, have $(GCC_VERSION), required 4.6)"
warning_libdaemon:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting OpenPRS Plugins$(TNORMAL) (libdaemon not found)"
$(WARN_TARGETS_BOOST): warning_boost_%:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting OpenPRS Plugin$(TNORMAL) (Boost library $* not found)"
endif

include $(BUILDSYSDIR)/base.mk

