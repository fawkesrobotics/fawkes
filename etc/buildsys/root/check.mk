#*****************************************************************************
#     Makefile Build System for Fawkes: Documentation config and targets
#                            -------------------
#   Created on Wed Mar 31 14:31:57 2010
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

ifndef __buildsys_root_docs_mk_
$(error docs.mk must be included before check.mk)
endif
ifndef __buildsys_config_mk_
$(error config.mk must be included before check.mk)
endif

ifndef __buildsys_root_check_mk_
__buildsys_root_check_mk_ := 1

.PHONY: license-check
license-check:
	$(SILENTSYMB)if which perl >/dev/null; then \
		perl $(FAWKES_BASEDIR)/etc/licscripts/find_invlic.pl -p src $(if $(SUBMODULE_EXTERN),-p fawkes/src) $(wildcard $(FAWKES_BASEDIR)/doc/headers/lichead*.*); \
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


endif # __buildsys_root_check_mk_

