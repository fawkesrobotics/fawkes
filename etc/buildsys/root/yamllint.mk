#*****************************************************************************
#     Makefile Build System for Fawkes: yamllint target
#                            -------------------
#   Created on Sat Jan 26 16:31:06 2019 +0100
#   Copyright (C) 2006-2019 by Tim Niemueller, AllemaniACs RoboCup Team
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
$(error config.mk must be included before docs.mk)
endif

ifndef __buildsys_root_yamllint_mk_
__buildsys_root_yamllint_mk_ := 1

.PHONY: yamllint

yamllint:
	$(SILENT) echo -e "$(INDENT_PRINT)[CHK] Checking configuration files"
	$(SILENT)if type -p yamllint >/dev/null; then \
		if ! yamllint -s cfg/config.yaml cfg/conf.d/; then \
			echo -e "$(TRED)--> yamllint reported issues, see above.$(TNORMAL)"; \
			exit 1; \
		else \
			echo -e "$(TGREEN)--> No warnings. Nice job.$(TNORMAL)"; \
		fi; \
	else \
		echo -e "$(TORANGE)--> yamllint could not be found$(TNORMAL)"; \
		exit 2; \
	fi

endif # __buildsys_root_yamllint_mk_

