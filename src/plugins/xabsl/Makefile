#*****************************************************************************
#            Makefile Build System for Fawkes: XABSL Plugin
#                            -------------------
#   Created on Wed Aug 06 14:24:52 2008
#   Copyright (C) 2006-2019 by Tim Niemueller [www.niemueller.org]
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
include $(BUILDSYSDIR)/download.mk

XABSLDIR=$(abspath $(SRCDIR)/xabsl-code/)
CFLAGS = -I$(SRCDIR)/XABSL/Xabsl -DXABSLDIR=\"$(XABSLDIR)/\"
XABSL_FILE_STEM = XABSL-Soccer
XABSL_FILE = $(XABSL_FILE_STEM).zip
URL_BASE=https://files.fawkesrobotics.org/projects/xabsl
URLS_xabsl = $(URL_BASE)/$(XABSL_FILE)

CLEAN_FILES=XABSL

all:

ifeq ($(OBJSSUBMAKE),1)

download-xabsl:
	$(SILENT)cd $(SRCDIR); \
	$(download-files)

$(SRCDIR)/XABSL: $(if $(wildcard $(SRCDIR)/$(XABSL_FILE)),,download-xabsl)
	$(SILENT)if [ ! -d $(SRCDIR)/XABSL ]; then \
		cd $(SRCDIR); \
		echo -e "$(INDENT_PRINT)--- Unpacking $(XABSL_FILE)"; \
		unzip -q -d $(SRCDIR)/XABSL $(XABSL_FILE); \
		echo -e "$(INDENT_PRINT)--- Patching XABSL lib for Fawkes compatibility"; \
		patch -s -d $(SRCDIR)/XABSL -p1 < $(SRCDIR)/$(XABSL_FILE_STEM).patch; \
		if type -p dos2unix >/dev/null 2>&1; then \
			dos2unix -q -k -o $(SRCDIR)/XABSL/Xabsl/compiler/xabsl.rb; \
		fi; \
	fi
endif

XABSL_SOURCES=Action Agent BooleanExpression DecimalExpression Engine \
	      EnumeratedExpression Option Parameters State Statement \
	      Symbols Tools
OBJS_libfawkesxabsl = $(addprefix XABSL/Xabsl/XabslEngine/Xabsl,$(addsuffix .o,$(XABSL_SOURCES)))
LIBS_all = $(LIBDIR)/libfawkesxabsl.so
LIBS_build = $(LIBS_all)

LIBS_xabsl = fawkesxabsl ObjectPositionInterface SkillerInterface
OBJS_xabsl = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.cpp))))
PLUGINS_all = $(PLUGINDIR)/xabsl.so
PLUGINS_build = $(PLUGINS_all)

OBJS_all = $(OBJS_libfawkesxabsl) $(OBJS_xabsl) $(XABSL_intermediate)

XABSL_intermediate = $(XABSLDIR)/agent.xabslc
TARGETS_all += $(XABSL_intermediate)

$(addprefix XABSL/Xabsl/XabslEngine/Xabsl,$(addsuffix .cpp,$(XABSL_SOURCES))): | $(SRCDIR)/XABSL

%.xabslc: %.xabsl
	$(SILENT)touch $(SRCDIR)/XABSL/Xabsl/compiler/timestamp
	$(SILENT)echo -e "$(INDENT_PRINT)--> Generating $(@F) (XABSL)"
	$(SILENT)ruby $(SRCDIR)/XABSL/Xabsl/compiler/xabsl.rb -q -I $(XABSLDIR) -i $@ $<

.PHONY: clean-xabsl
clean-xabsl:
	$(SILENT)rm -rf $(XABSLDIR)/*.xabslc $(XABSLDIR)/*.dump
clean: clean-xabsl

$(PLUGINS_build): | $(LIBS_build)

include $(BUILDSYSDIR)/base.mk
