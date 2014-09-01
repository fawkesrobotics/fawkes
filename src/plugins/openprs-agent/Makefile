#*****************************************************************************
#          Makefile Build System for Fawkes: OpenPRS Agent Plugin
#                            -------------------
#   Created on Fri Aug 22 13:54:23 2014
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
include $(BUILDSYSDIR)/openprs.mk

LIBS_openprs_agent = fawkescore fawkesutils fawkesaspects fawkesblackboard \
                     fawkesinterface fawkesopenprsaspect fawkesopenprsutils
OBJS_openprs_agent = openprs_agent_plugin.o openprs_agent_thread.o

LIBS_openprs_mod_skiller = fawkescore fawkesutils fawkesblackboard \
	                  fawkesinterface SkillerInterface
OBJS_openprs_mod_skiller = mod_skiller.o

LIBS_openprs_mod_time = fawkescore fawkesutils
OBJS_openprs_mod_time = mod_time.o

LIBS_openprs_mod_utils = fawkescore fawkesutils
OBJS_openprs_mod_utils = mod_utils.o

OBJS_all = $(OBJS_openprs_agent) $(OBJS_openprs_mod_skiller) \
	   $(OBJS_openprs_mod_time) $(OBJS_openprs_mod_utils)

ifeq ($(HAVE_OPENPRS),1)
  CFLAGS  += $(CFLAGS_OPENPRS)
  LDFLAGS += $(LDFLAGS_OPENPRS)

  CFLAGS_mod_skiller = $(CFLAGS_OPENPRS) $(CFLAGS_OPENPRS_MOD)
  LDFLAGS_openprs_mod_skiller = $(LDFLAGS_OPENPRS) $(LDFLAGS_OPENPRS_MOD)
  CFLAGS_mod_time    = $(CFLAGS_mod_skiller)
  LDFLAGS_openprs_mod_time = $(LDFLAGS_openprs_mod_skiller)
  CFLAGS_mod_utils    = $(CFLAGS_mod_skiller)
  LDFLAGS_openprs_mod_utils = $(LDFLAGS_openprs_mod_skiller)
  ALLOW_UNDEF_openprs_mod_skiller = 1
  ALLOW_UNDEF_openprs_mod_time = 1
  ALLOW_UNDEF_openprs_mod_utils = 1

  PLUGINS_all = $(PLUGINDIR)/openprs-agent.so
  LIBS_all   += $(OPENPRS_MOD_DIR)/mod_skiller.so $(OPENPRS_MOD_DIR)/mod_time.so \
		$(OPENPRS_MOD_DIR)/mod_utils.so
else
  WARN_TARGETS += warning_openprs
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_openprs
warning_openprs:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting OpenPRS agent plugin$(TNORMAL) ($(OPENPRS_ERROR))"
endif

include $(BUILDSYSDIR)/base.mk

