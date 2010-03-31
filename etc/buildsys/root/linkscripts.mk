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

TARGETS_all += linkscripts

.PHONY: linkscripts
linkscripts:
	$(SILENT) if [ -d $(BASEDIR)/etc/scripts ]; then \
		mkdir -p $(BINDIR); \
		for f in $$(ls $(BASEDIR)/etc/scripts); do \
			if [ -x "$(BASEDIR)/etc/scripts/$$f" ]; then \
				if [[ -a "$(BINDIR)/$$f" && ! -L "$(BINDIR)/$$f" ]]; then \
					echo -e "$(INDENT_PRINT)$(TRED)--- Non-symbolic link bin/$$f exists, *not* linking to etc/scripts/$$f$(TNORMAL)"; \
				else \
					echo -e "$(INDENT_PRINT)--- Linking bin/$$f -> etc/scripts/$$f"; \
					rm -f $(BINDIR)/$$f; \
					ln -s $(BASEDIR)/etc/scripts/$$f $(BINDIR)/$$f; \
				fi \
			else \
				echo -e "$(INDENT_PRINT)$(TYELLOW)--- Omitting etc/scripts/$$f (not executable)$(TNORMAL)"; \
			fi \
		done; \
	fi


endif # __buildsys_root_linkscripts_mk_

