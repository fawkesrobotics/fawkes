#*****************************************************************************
#            Makefile Build System for Fawkes: XABSL Plugin
#                            -------------------
#   Created on Wed Aug 06 14:24:52 2008
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
include $(BUILDSYSDIR)/download.mk

XABSLDIR=$(abspath $(SRCDIR)/xabsl-code/)
CFLAGS = -I$(SRCDIR)/XABSL/Xabsl -DXABSLDIR=\"$(XABSLDIR)/\"
XABSL_FILE_STEM = XABSL-Soccer
XABSL_FILE = $(XABSL_FILE_STEM).zip
URLS_xabsl = http://robocup.rwth-aachen.de/files/sourcecode/$(XABSL_FILE) \
	     http://robocup.rwth-aachen.de/files/sourcecode/$(XABSL_FILE_STEM).patch

all:

ifeq ($(OBJSSUBMAKE),1)
all: get-xabsl

download-xabsl:
	$(SILENT)cd $(SRCDIR); \
	$(download-files)

.PHONY: get-xabsl
get-xabsl: $(if $(wildcard $(addprefix $(SRCDIR)/,$(notdir $(URLS_xabsl)))),,download-xabsl)
	$(SILENT)if [ ! -d $(SRCDIR)/XABSL ]; then \
		cd $(SRCDIR); \
		echo -e "$(INDENT_PRINT)--- Unpacking $(XABSL_FILE)"; \
		unzip -q -d $(SRCDIR)/XABSL $(XABSL_FILE); \
		echo -e "$(INDENT_PRINT)--- Patching XABSL lib for Fawkes compatibility"; \
		patch -s -d $(SRCDIR)/XABSL -p1 < $(SRCDIR)/$(XABSL_FILE_STEM).patch; \
	fi
endif

XABSL_SOURCES=Action Agent BooleanExpression DecimalExpression Engine \
	      EnumeratedExpression Option Parameters State Statement \
	      Symbols Tools
OBJS_libxabsl = $(addprefix XABSL/Xabsl/XabslEngine/Xabsl,$(addsuffix .o,$(XABSL_SOURCES)))
LIBS_all = $(LIBDIR)/libxabsl.so

LIBS_xabsl = xabsl ObjectPositionInterface SkillerInterface
OBJS_xabsl = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.cpp))))
PLUGINS_all = $(PLUGINDIR)/xabsl.so

XABSL_intermediate = $(XABSLDIR)/agent.xabslc
TARGETS_all += $(XABSL_intermediate)

%.xabslc: %.xabsl
	$(SILENT)touch $(SRCDIR)/XABSL/Xabsl/compiler/timestamp
	$(SILENT)echo -e "$(INDENT_PRINT)--> Generating $(@F) (XABSL)"
	$(SILENT)ruby $(SRCDIR)/XABSL/Xabsl/compiler/xabsl.rb -q -I $(XABSLDIR) -i $@ $<

.PHONY: clean-xabsl
clean-xabsl:
	$(SILENT)rm -rf $(XABSLDIR)/*.xabslc $(XABSLDIR)/*.dump
clean: clean-xabsl

$(PLUGINS_all): | $(LIBS_all)

OBJS_all = $(OBJS_libxabsl) $(OBJS_xabsl) $(XABSL_intermediate)

include $(BUILDSYSDIR)/base.mk
