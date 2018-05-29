#*****************************************************************************
#          Makefile Build System for Fawkes: Base Application Library
#                            -------------------
#   Created on Wed May 04 21:20:58 2011
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

BASEDIR = ../../..
include $(BASEDIR)/etc/buildsys/config.mk
include $(BUILDCONFDIR)/tf/tf.mk
include $(BUILDCONFDIR)/logging/logging.mk
include $(BUILDSYSDIR)/boost.mk

CFLAGS  += $(CFLAGS_LIBDAEMON) $(CFLAGS_TF)
LDFLAGS += $(LDFLAGS_LIBDAEMON) $(LDFLAGS_TF)

LIBS_libfawkesbaseapp = stdc++ pthread fawkescore fawkesutils fawkesconfig \
			fawkesblackboard fawkesplugin fawkesnetcomm \
			fawkesaspects fawkeslogging fawkesnetworklogger \
			fawkessyncpoint \

REQ_BOOST_LIBS = asio system
HAVE_BOOST_LIBS = $(call boost-have-libs,$(REQ_BOOST_LIBS))

ifeq ($(HAVE_TF),1)
  LIBS_libfawkesbaseapp += fawkestf
else
  WARN_TARGETS += warning_tf
endif

ifeq ($(HAVE_CPP11)$(HAVE_BOOST_LIBS),11)
  CFLAGS  += -DHAVE_BOOST -DHAVE_CPP11 $(CFLAGS_CPP11) $(call boost-libs-cflags,$(REQ_BOOST_LIBS))
  LDFLAGS += $(call boost-libs-ldflags,$(REQ_BOOST_LIBS))
endif
ifeq ($(HAVE_LOGGING_FD_REDIRECT),1)
  CFLAGS  += -DHAVE_LOGGING_FD_REDIRECT
else
  WARN_TARGETS += warning_fd_redirect
endif

OBJS_libfawkesbaseapp =	$(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))
HDRS_libfawkesbaseapp = $(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.h))

OBJS_all = $(OBJS_libfawkesbaseapp)
LIBS_all  = $(LIBDIR)/libfawkesbaseapp.so

ifeq ($(OBJSSUBMAKE),1)
  ifneq ($(WARN_TARGETS),)
all: $(WARN_TARGETS)
  endif
.PHONY: warning_libdaemon warning_tf
warning_libdaemon:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)Building without daemonizing support$(TNORMAL) (libdaemon[-devel] not installed)"
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)Building without TF support$(TNORMAL) (tf framework not available)"
warning_fd_redirect:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)Building stdout/stderr redirect logger support$(TNORMAL) (feature not available in logging lib)"
endif

include $(BUILDSYSDIR)/base.mk

