#*****************************************************************************
#            Makefile Build System for Fawkes: Webview Frontend
#                            -------------------
#   Created on Fri Mar 30 20:23:49 2018
#   Copyright (C) 2006-2018 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

BASEDIR = ../../../..

include $(BASEDIR)/etc/buildsys/config.mk

NG_CLI=$(shell type -p ng)
NPM=$(shell type -p npm)

ifneq ($(NG_CLI),)
  ifneq ($(NPM),)
		TARGETS_all += dist/index.html

		SRCS_FRONTEND = $(shell find $(SRCDIR)/src/ -regex '.*\.\(html\|ts\|json\|js\|scss\)')
else
    WARN_TARGETS += warning_npm
  endif
else
    WARN_TARGETS += warning_ng_cli
endif

dist/index.html: $(SRCS_FRONTEND) $(SRCDIR)/node_modules
	$(SILENT) echo -e "$(INDENT_PRINT)[Angular] $(TBOLDGRAY)Webview Frontend$(TNORMAL)"
	$(SILENT)ng build --prod --build-optimizer

$(SRCDIR)/node_modules:
	$(SILENT) echo -e "$(INDENT_PRINT)[npm] $(TBOLDGRAY)Installing dependencies via npm$(TNORMAL)"
	$(SILENT)npm install

deploy: $(RESDIR)/webview/index.html

clean: clean-webview

distclean: distclean-webview

clean-webview:
	$(SILENT) echo -e "$(INDENT_PRINT)[CLN] Clean $(TBOLDGRAY)$(subst $(abspath $(TOP_BASEDIR))/,,$(RESDIR)/webview)$(TNORMAL)"
	$(SILENT)rm -rf $(RESDIR)/webview/*

distclean-webview: clean
	$(SILENT) echo -e "$(INDENT_PRINT)[CLEAN] Clean $(TBOLDGRAY)node_modules$(TNORMAL)"
	$(SILENT)rm -rf $(SRCDIR)/node_modules


$(RESDIR)/webview/index.html: dist/index.html
	$(SILENT) echo -e "$(INDENT_PRINT)[ng-depl] dist -> $(TBOLDGRAY)$(patsubst $(realpath $(BASEDIR))/%,%,$(@D))$(TNORMAL)"
	$(SILENT)cp -a dist/* $(RESDIR)/webview/

ifeq ($(OBJSSUBMAKE),1)
ifneq ($(WARN_TARGETS),)
all: $(WARN_TARGETS)
  endif
.PHONY: warning_npm warning_ng_cli
warning_npm:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build webview frontend$(TNORMAL) (npm not found)"
warning_ng_cli:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build webview frontend$(TNORMAL) (Angular CLI ng not found)"
endif

include $(BUILDSYSDIR)/rules.mk
