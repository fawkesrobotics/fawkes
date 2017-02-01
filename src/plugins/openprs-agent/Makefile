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

LIBS_openprs_mod_protobuf = fawkescore fawkesutils fawkes_protobuf_comm
OBJS_openprs_mod_protobuf = oprs_protobuf.o mod_protobuf.o

LIBS_openprs_mod_navgraph = fawkescore fawkesnavgraph fawkesconfig fawkesnetcomm
OBJS_openprs_mod_navgraph = mod_navgraph.o

OBJS_all = $(OBJS_openprs_agent) $(OBJS_openprs_mod_skiller) \
	   $(OBJS_openprs_mod_protobuf) $(OBJS_openprs_mod_navgraph)

INST_LIB_SUBDIR_openprs_mod_skiller  = $(FFLIBSUBDIR)
INST_LIB_SUBDIR_openprs_mod_protobuf = $(FFLIBSUBDIR)
INST_LIB_SUBDIR_openprs_mod_navgraph = $(FFLIBSUBDIR)
NOSOVER_openprs_mod_skiller  = 1
NOSOVER_openprs_mod_protobuf = 1
NOSOVER_openprs_mod_navgraph = 1

# Extra files for system-wide install, i.e., "make install"
FILES_openprs_files   = $(subst $(SRCDIR)/,,$(wildcard $(addprefix $(SRCDIR)/data/,*.opf *.inc *.sym)))
DESTDIR_openprs_files = $(EXEC_SHAREDIR)/openprs/openprs-agent

ifeq ($(HAVE_OPENPRS),1)
  CFLAGS  += $(CFLAGS_OPENPRS)
  LDFLAGS += $(LDFLAGS_OPENPRS)

  CFLAGS_mod_skiller = $(CFLAGS_OPENPRS) $(CFLAGS_OPENPRS_MOD)
  LDFLAGS_openprs_mod_skiller = $(LDFLAGS_OPENPRS) $(LDFLAGS_OPENPRS_MOD) -lpu
  CFLAGS_mod_protobuf = $(CFLAGS_mod_skiller)
  CFLAGS_oprs_protobuf = $(CFLAGS_mod_skiller)
  CFLAGS_mod_navgraph = $(CFLAGS_mod_skiller)
  CFLAGS_oprs_navgraph = $(CFLAGS_mod_skiller)
  LDFLAGS_openprs_mod_protobuf = $(LDFLAGS_openprs_mod_skiller)
  LDFLAGS_openprs_mod_navgraph = $(LDFLAGS_openprs_mod_skiller)
  ALLOW_UNDEF_openprs_mod_skiller = 1
  ALLOW_UNDEF_openprs_mod_protobuf = 1
  ALLOW_UNDEF_openprs_mod_navgraph = 1

  PLUGINS_all = $(PLUGINDIR)/openprs-agent.so
  LIBS_all   += $(OPENPRS_MOD_DIR)/mod_skiller.so \
		$(OPENPRS_MOD_DIR)/mod_protobuf.so \
		$(OPENPRS_MOD_DIR)/mod_navgraph.so

	INSTALL_extra = openprs_files
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

