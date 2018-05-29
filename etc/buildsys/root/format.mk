#*****************************************************************************
#     Makefile Build System for Fawkes: Formatting targets
#                            -------------------
#   Created on Sun May 27 20:14:54 2018
#   Copyright (C) 2006-2018 by Tim Niemueller [www.niemueller.de]
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
$(error config.mk must be included before format.mk)
endif

ifndef __buildsys_root_format_mk_
__buildsys_root_format_mk_ := 1

.PHONY: format-clang
format: check-parallel
	$(SILENT) echo -e "$(INDENT_PRINT)[FMT] Formatting where necessary"
	$(SILENTSYMB)if type -p clang-format >/dev/null; then \
		ALL_FILES=$$(git ls-files *.{h,cpp}); \
		if type -p parallel >/dev/null; then \
			parallel -u --will-cite --bar clang-format -i ::: $$ALL_FILES; \
		else \
			echo "$$ALL_FILES" | xargs -P$$(nproc) -n1 clang-format -i; \
		fi; \
	else \
		echo -e "$(INDENT_PRINT)$(TRED)--- Cannot format code$(TNORMAL) (clang-format not found)"; \
    exit 1; \
  fi

# Using clang-format is the default, faster and better results
.PHONY: format
format: format-clang

endif # __buildsys_root_format_mk_
