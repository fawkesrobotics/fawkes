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
ifneq ($(wildcard $(BUILDCONFDIR)/tf/tf.mk),)
  include $(BUILDCONFDIR)/tf/tf.mk
endif
ifneq ($(wildcard $(BUILDCONFDIR)/logging/logging.mk),)
  include $(BUILDCONFDIR)/logging/logging.mk
endif
include $(BUILDSYSDIR)/boost.mk

CFLAGS  += $(CFLAGS_LIBDAEMON)
LDFLAGS += $(LDFLAGS_LIBDAEMON)

LIBS_libfawkesbaseapp = stdc++ pthread fawkescore fawkesutils fawkesconfig \
			fawkesplugin fawkesaspects fawkeslogging \
			fawkessyncpoint

REQ_BOOST_LIBS = asio system
HAVE_BOOST_LIBS = $(call boost-have-libs,$(REQ_BOOST_LIBS))

ifneq ($(wildcard $(SRCDIR)/../blackboard/blackboard.h),)
  CFLAGS += -DHAVE_BLACKBOARD
  LIBS_libfawkesbaseapp += fawkesblackboard
  ifeq ($(HAVE_TF),1)
    CFLAGS  += $(CFLAGS_TF)
    LDFLAGS += $(LDFLAGS_TF)
    LIBS_libfawkesbaseapp += fawkestf
  else
    WARN_TARGETS += warning_tf
  endif
else
  WARN_TARGETS += warning_blackboard
endif

ifneq ($(wildcard $(SRCDIR)/../netcomm/fawkes/network_manager.h),)
  CFLAGS += -DHAVE_NETWORK_MANAGER
  LIBS_libfawkesbaseapp += fawkesnetcomm
  ifneq ($(wildcard $(SRCDIR)/../network_logger/network_logger.h),)
    CFLAGS += -DHAVE_NETWORK_LOGGER
    LIBS_libfawkesbaseapp += fawkesnetworklogger
  else
    WARN_TARGETS += warning_network_logger
  endif
  ifneq ($(wildcard $(SRCDIR)/../config/net_handler.h),)
    CFLAGS += -DHAVE_CONFIG_NETWORK_HANDLER
  else
    WARN_TARGETS += warning_config_net_handler
  endif
  ifneq ($(wildcard $(SRCDIR)/../plugin/net/handler.h),)
    CFLAGS += -DHAVE_PLUGIN_NETWORK_HANDLER
  else
    WARN_TARGETS += warning_plugin_net_handler
  endif
else
  WARN_TARGETS += warning_netcomm
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
LIBS_build = $(LIBS_all)

ifeq ($(OBJSSUBMAKE),1)
  ifneq ($(WARN_TARGETS),)
all: $(WARN_TARGETS)
  endif
.PHONY: warning_libdaemon warning_tf warning_blackboard warning_netcomm warning_config_net_handler warning_plugin_net_handler warning_network_logger
warning_libdaemon:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)Building without daemonizing support$(TNORMAL) (libdaemon[-devel] not installed)"
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)Building without TF support$(TNORMAL) (tf framework not available)"
warning_fd_redirect:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)Building stdout/stderr redirect logger support$(TNORMAL) (feature not available in logging lib)"
warning_blackboard:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)Building without blackboard support$(TNORMAL) (blackboard not available)"
warning_netcomm:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)Building without Fawkes network support$(TNORMAL) (netcomm library not available)"
warning_config_net_handler:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)Building without config network handler support$(TNORMAL) (config network handler not available)"
warning_plugin_net_handler:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)Building without plugin network handler support$(TNORMAL) (plugin network handler not available)"
warning_network_logger:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)Building without network logger support$(TNORMAL) (network logger not available)"
endif

include $(BUILDSYSDIR)/base.mk

