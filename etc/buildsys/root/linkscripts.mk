#*****************************************************************************
#     Makefile Build System for Fawkes: Documentation config and targets
#                            -------------------
#   Created on Wed Mar 31 15:36:39 2010
#   Copyright (C) 2006-2010 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

ifndef __buildsys_config_mk_
$(error config.mk must be included before linkscripts.mk)
endif

ifndef __buildsys_root_linkscripts_mk_
__buildsys_root_linkscripts_mk_ := 1

all: linkscripts

SCRIPTS = $(BASEDIR)/etc/scripts
ifneq ($(abspath $(BASEDIR)),$(abspath $(FAWKES_BASEDIR)))
SCRIPTS += $(FAWKES_BASEDIR)/etc/scripts
endif

.PHONY: linkscripts
linkscripts:
	$(SILENT) if [ -d $(BASEDIR)/etc/scripts ]; then \
		mkdir -p $(BINDIR); \
		for path in $(SCRIPTS); do \
			for f in $$(ls $${path}); do \
				if [ -x "$${path}/$$f" ]; then \
					if [[ -a "$(BINDIR)/$$f" && ! -L "$(BINDIR)/$$f" ]]; then \
						echo -e "$(INDENT_PRINT)$(TRED)[ERR] Non-symbolic link bin/$$f exists, *not* linking to etc/scripts/$$f$(TNORMAL)"; \
					else \
						echo -e "$(INDENT_PRINT)[LNK] bin/$$f -> $${path}/$$f"; \
						rm -f $(BINDIR)/$$f; \
						ln -s ../$${path}/$$f $(BINDIR)/$$f; \
					fi \
				else \
					echo -e "$(INDENT_PRINT)$(TYELLOW)[WARN] Omitting $${path}/$$f (not executable)$(TNORMAL)"; \
				fi \
			done; \
		done; \
	fi


endif # __buildsys_root_linkscripts_mk_

