#*****************************************************************************
#                      Makefile Build System for Fawkes
#                            -------------------
#   Created on Wed Sep 06 02:39:46 2006
#   Copyright (C) 2006 by Tim Niemueller, AllemaniACs RoboCup Team
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

BASEDIR = .

SUBDIRS = src

TARGETS_all += linkscripts

include $(BASEDIR)/etc/buildsys/config.mk
include $(BASEDIR)/etc/buildsys/rules.mk
include $(BASEDIR)/etc/buildsys/lua.mk

.PHONY: apidoc quickdoc tracdoc uncolored-quickdoc
apidoc: api.doxygen
quickdoc: api-quick.doxygen
tracdoc: api-trac.doxygen
uncolored-quickdoc: quickdoc

%.doxygen:
	$(SILENT) echo "--> Building documentation ($@). This may take a while..."
	$(SILENT) rm -rf doc/api
	$(SILENT) $(DOXYGEN) doc/doxygen/$@ >/dev/null 2>&1
	$(SILENT) if [ "`wc -l warnings.txt | awk '{ print $$1 }'`" != "0" ]; then \
		$(RED); \
		echo "--> Warnings have been generated:"; \
		$(NORMAL); \
		cat warnings.txt; \
		exit 1; \
	else \
		$(GREEN); \
		echo "--> No warnings. Nice job."; \
		$(NORMAL); \
	fi

.PHONY: linkscripts
linkscripts:
	$(SILENT) for f in $$(ls $(BASEDIR)/etc/scripts); do \
		if [ -e "$(BASEDIR)/etc/scripts/$$f" ]; then \
			if [[ -a "$(BASEDIR)/bin/$$f" && ! -L "$(BASEDIR)/bin/$$f" ]]; then \
				echo -e "$(INDENT_PRINT)$(TRED)--- Non-symbolic link bin/$$f exists, *not* linking to etc/scripts/$$f$(TNORMAL)"; \
			else \
				echo -e "$(INDENT_PRINT)--- Linking bin/$$f -> etc/scripts/$$f"; \
				rm -f bin/$$f; \
				ln -s ../etc/scripts/$$f bin; \
			fi \
		fi \
	done

.PHONY: license-check uncolored-license-check
uncolored-license-check: license-check
license-check:
	$(SILENT)if which perl >/dev/null; then \
		perl $(BASEDIR)/etc/licscripts/find_invlic.pl src $(wildcard $(BASEDIR)/doc/headers/lichead*.*); \
		if [ $$? = 0 ]; then \
			echo -e "$(INDENT_PRINT)$(TGREEN)--> All source files have a proper license header.$(TNORMAL)"; \
		else \
			echo -e "$(INDENT_PRINT)$(TRED)--> Some source files do not have a proper license header. Fix it!$(TNORMAL)"; \
			exit 1; \
		fi \
	else \
		echo -e "$(INDENT_PRINT)$(TRED)--- Cannot do license check$(TNORMAL) (perl not installed)"; \
		exit 1; \
	fi

.PHONY: simple-clean
simple-clean:
	$(SILENT)rm -f $(BINDIR)/* $(LIBDIR)/*.so $(if $(LUALIBDIR),$(LUALIBDIR)/*.so) $(PLUGINDIR)/*.so

.PHONY: switch-buildtype
switch-buildtype: simple-clean
	$(SILENT) if [ -z "$(BT)" ]; then \
		echo -e "$(INDENT_PRINT)$(TRED)--- Usage: make build-type BT=new_buildtype$(TNORMAL)"; \
	else \
		echo -e "$(INDENT_PRINT)--- Switching build type from $(BUILD_TYPE) to $(BT)"; \
		sed -i -e 's/^BUILD_TYPE=.*$$/BUILD_TYPE=$(BT)/' etc/buildsys_local/buildtype.mk; \
	fi

