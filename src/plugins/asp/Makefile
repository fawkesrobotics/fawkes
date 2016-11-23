#*****************************************************************************
#               Makefile Build System for Fawkes: ASP Plugin
#                            -------------------
#   Created on Thu Oct 20 15:00:34 2016
#   Copyright (C) 2016 by Björn Schäpers
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
#include $(BUILDSYSDIR)/clips.mk asp.mk?

LIBS_asp = fawkescore fawkesutils fawkesaspects fawkesblackboard \
		   fawkesinterface fawkesaspaspect
OBJS_asp = asp_plugin.o asp_thread.o

OBJS_all = $(OBJS_asp)

#TODO: export this in .mk and really check
HAVE_CLINGO = 1
CFLAGS_CLINGO = -DWITH_THREADS=1 \
	-I $(HOME)/clingo/libclasp \
	-I $(HOME)/clingo/libclingo \
	-I $(HOME)/clingo/libgringo \
	-I $(HOME)/clingo/liblp \
	-I $(HOME)/clingo/libprogram_opts
LDFLAGS_CLINGO = -L$(HOME)/clingo-build/bin/ -Wl,-rpath=$(HOME)/clingo-build/bin/ -lclingo

# Extra files for system-wide install, i.e., "make install"
#FILES_clips_files   = $(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/clips/*.clp))
#DESTDIR_clips_files = $(EXEC_SHAREDIR)/clips/clips

ifeq ($(HAVE_CPP11),1)
  ifeq ($(HAVE_CLINGO),1)
    PRESUBDIRS += aspect

    CFLAGS  += $(CFLAGS_CLINGO) $(CFLAGS_CPP11)
    LDFLAGS += $(LDFLAGS_CLINGO) $(LDFLAGS_CPP11)

    PLUGINS_all = $(PLUGINDIR)/asp.so

    INSTALL_extra = asp_files
  else
    WARN_TARGETS += warning_clingo
  endif
else
  WARN_TARGETS += warning_cpp11
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_clingo warning_cpp11
warning_clingo:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ASP Plugin$(TNORMAL) ($(CLINGO_ERROR))"
warning_cpp11:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ASP Plugin$(TNORMAL) (C++11 not supported)"
endif

include $(BUILDSYSDIR)/base.mk

