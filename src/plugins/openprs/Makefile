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
include $(BUILDSYSDIR)/openprs.mk

LIBS_openprs = fawkescore fawkesutils fawkesaspects fawkesblackboard \
               fawkesinterface fawkesopenprsaspect fawkesopenprsutils \
	       fawkesbaseapp fawkesnetcomm
OBJS_openprs = openprs_plugin.o openprs_thread.o

LIBS_openprs_mod_utils = fawkescore fawkesutils
OBJS_openprs_mod_utils = mod_utils.o

LIBS_openprs_mod_time = fawkescore fawkesutils
OBJS_openprs_mod_time = mod_time.o

LIBS_openprs_mod_blackboard = fawkescore fawkesutils fawkesblackboard \
			      fawkesinterface
OBJS_openprs_mod_blackboard = mod_blackboard.o

LIBS_openprs_mod_config = fawkescore fawkesutils fawkesconfig fawkesnetcomm
OBJS_openprs_mod_config = mod_config.o

OBJS_all = $(OBJS_openprs) $(OBJS_openprs_mod_utils) \
	   $(OBJS_openprs_mod_time) $(OBJS_openprs_mod_blackboard) \
	   $(OBJS_openprs_mod_config)

INST_LIB_SUBDIR_openprs_mod_utils = $(FFLIBSUBDIR)
INST_LIB_SUBDIR_openprs_mod_time = $(FFLIBSUBDIR)
INST_LIB_SUBDIR_openprs_mod_blackboard = $(FFLIBSUBDIR)
INST_LIB_SUBDIR_openprs_mod_config = $(FFLIBSUBDIR)
NOSOVER_openprs_mod_utils = 1
NOSOVER_openprs_mod_time = 1
NOSOVER_openprs_mod_blackboard = 1
NOSOVER_openprs_mod_config = 1

# Extra files for system-wide install, i.e., "make install"
FILES_openprs_files   = $(subst $(SRCDIR)/,,$(wildcard $(addprefix $(SRCDIR)/data/,*.opf *.inc)))
DESTDIR_openprs_files = $(EXEC_SHAREDIR)/openprs/data

ifeq ($(HAVE_OPENPRS),1)
  PRESUBDIRS += utils aspect
  SUBDIRS    += example

  CFLAGS  += $(CFLAGS_OPENPRS)  $(CFLAGS_OPENPRS_MP) 
  LDFLAGS += $(LDFLAGS_OPENPRS) $(LDFLAGS_OPENPRS_MP)

  CFLAGS_mod_utils    = $(CFLAGS_mod_skiller)
  LDFLAGS_openprs_mod_utils = $(LDFLAGS_openprs_mod_skiller)
  CFLAGS_mod_time    = $(CFLAGS_mod_skiller)
  LDFLAGS_openprs_mod_time = $(LDFLAGS_openprs_mod_skiller)
  CFLAGS_mod_blackboard = $(CFLAGS_mod_skiller)
  LDFLAGS_openprs_mod_blackboard = $(LDFLAGS_openprs_mod_skiller)
  CFLAGS_mod_config = $(CFLAGS_mod_skiller)
  LDFLAGS_openprs_mod_config = $(LDFLAGS_openprs_mod_skiller)
  ALLOW_UNDEF_openprs_mod_utils = 1
  ALLOW_UNDEF_openprs_mod_time = 1
  ALLOW_UNDEF_openprs_mod_blackboard = 1
  ALLOW_UNDEF_openprs_mod_config = 1

  PLUGINS_all = $(PLUGINDIR)/openprs.so
  LIBS_all   += $(OPENPRS_MOD_DIR)/mod_utils.so \
		$(OPENPRS_MOD_DIR)/mod_time.so \
		$(OPENPRS_MOD_DIR)/mod_blackboard.so \
		$(OPENPRS_MOD_DIR)/mod_config.so

	INSTALL_extra = openprs_files
else
  WARN_TARGETS += warning_openprs
endif

aspect: utils
example: aspect utils

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_openprs
warning_openprs:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting OpenPRS Aspects$(TNORMAL) ($(OPENPRS_ERROR))"
endif

include $(BUILDSYSDIR)/base.mk

