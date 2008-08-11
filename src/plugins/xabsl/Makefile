#*****************************************************************************
#            Makefile Build System for Fawkes: XABSL Plugin
#                            -------------------
#   Created on Wed Aug 06 14:24:52 2008
#   Copyright (C) 2006-2008 by Tim Niemueller, AllemaniACs RoboCup Team
#
#   $Id$
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
include $(BASEDIR)/etc/buildsys/download.mk

CFLAGS = -I$(SRCDIR)/XABSL/Xabsl -DXABSLDIR=\"$(SRCDIR)/xabsl-code/\"
XABSL_FILE_STEM = XABSL-Soccer
XABSL_FILE = $(XABSL_FILE_STEM).zip
URLS_xabsl = http://robocup.rwth-aachen.de/files/sourcecode/$(XABSL_FILE) \
	     http://robocup.rwth-aachen.de/files/sourcecode/$(XABSL_FILE_STEM).patch

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

LIBS_xabsl = xabsl
OBJS_xabsl = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.cpp))))
PLUGINS_all = $(PLUGINDIR)/xabsl.so

$(PLUGINS_all): | $(LIBS_all)

OBJS_all = $(OBJS_libxabsl) $(OBJS_xabsl)

include $(BASEDIR)/etc/buildsys/base.mk
