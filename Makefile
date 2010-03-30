#*****************************************************************************
#                      Makefile Build System for Fawkes
#                            -------------------
#   Created on Wed Sep 06 02:39:46 2006
#   Copyright (C) 2006 by Tim Niemueller, AllemaniACs RoboCup Team
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

include $(BASEDIR)/etc/buildsys/config.mk
include $(BUILDSYSDIR)/rules.mk
include $(BUILDSYSDIR)/lua.mk

.PHONY: apidoc quickdoc tracdoc
apidoc: api.doxygen
quickdoc: api-quick.doxygen
tracdoc: api-trac.doxygen

%.doxygen:
	$(SILENTSYMB) echo "--> Building documentation ($@). This may take a while..."
	$(SILENT) rm -rf doc/api
	$(SILENT) $(DOXYGEN) doc/doxygen/$@ >/dev/null 2>&1
	$(SILENTSYMB) if [ "`wc -l warnings.txt | awk '{ print $$1 }'`" != "0" ]; then \
		echo -e "$(TRED)--> Warnings have been generated:$(TNORMAL)"; \
		cat warnings.txt; \
		exit 1; \
	else \
		echo -e "$(TGREEN)--> No warnings. Nice job.$(TNORMAL)"; \
	fi

# Uncolored implicit targets
uncolored-%: % ; @:

.PHONY: license-check
license-check:
	$(SILENTSYMB)if which perl >/dev/null; then \
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

.PHONY: check
check: quickdoc license-check

.PHONY: simple-clean
simple-clean:
	$(SILENT)rm -f $(BINDIR)/* $(LIBDIR)/*.so $(if $(LUALIBDIR),$(LUALIBDIR)/*.so) $(PLUGINDIR)/*.so

.PHONY: switch-buildtype print-buildtype
switch-buildtype:
	$(SILENT) if [ -z "$(BT)" ]; then \
		echo -e "$(INDENT_PRINT)$(TRED)--- Usage: make switch-buildtype BT=new_buildtype$(TNORMAL)"; \
		echo -e "$(INDENT_PRINT)$(TRED)---    or: make switch-buildtype-new_buildtype$(TNORMAL)"; \
	elif [ "$(BUILD_TYPE)" = "$(BT)" ]; then \
		echo -e "$(INDENT_PRINT)$(TYELLOW)--- Build type $(BT) is already set$(TNORMAL)"; \
	else \
		echo -e "$(INDENT_PRINT)--- Switching build type from $(BUILD_TYPE) to $(BT)"; \
		sed -i -e 's/^BUILD_TYPE=.*$$/BUILD_TYPE=$(BT)/' etc/buildsys/buildtype.mk; \
		for D in $(BINDIR) $(LIBDIR) $(PLUGINDIR); do \
			rm -rf $${D}_$(BUILD_TYPE); \
			mv $$D $${D}_$(BUILD_TYPE); \
			if [ -d $${D}_$(BT) ]; then \
				mv $${D}_$(BT) $$D; \
			else \
				mkdir $$D; \
			fi; \
			find $${D} -name .svn -type d -prune -exec rm -rf {} \; ; \
			if [ -d $${D}_$(BUILD_TYPE) ]; then \
				SVNDIRS=`find $${D}_$(BUILD_TYPE) -name .svn -type d -prune`; \
				for d in $$SVNDIRS; do \
					t=$${d#$${D}_$(BUILD_TYPE)/}; \
					t=$$t; \
					mkdir -p $${D}/$${t%.svn}; \
					rm -rf $${D}/$$t; \
					mv $${D}_$(BUILD_TYPE)/$$t $${D}/$$t; \
				done; \
			fi; \
		done; \
	fi

# Easier to remember target than having to remember BT variable
switch-buildtype-%:
	$(SILENT)$(MAKE) --no-print-directory switch-buildtype BT=$* 

print-buildtype:
	$(SILENTSYMB) echo -e "$(INDENT_PRINT)--- Current build type: $(BUILD_TYPE)";

