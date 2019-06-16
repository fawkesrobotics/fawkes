
#*****************************************************************************
#     Makefile Build System for Fawkes: Local formatting targets
#                            -------------------
#   Created on Fri Mar 08 10:08:43 2019 +0100
#   Copyright (C) 2006-2019 by Tim Niemueller [www.niemueller.de]
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

.PHONY: format-modified-clang
format-modified-clang:
	$(SILENT) echo -e "$(INDENT_PRINT)[FMT] Formatting modified files (git)"
	$(SILENTSYMB)if type -p clang-format >/dev/null; then \
		pushd $(SRCDIR) >/dev/null; \
		MODIFIED_FILES=$$(git ls-files -m \*.{h,cpp}); \
		UNTRACKED_FILES=$$(git ls-files --exclude-standard --others \*.{h,cpp}); \
		FILES=$$(echo "$$MODIFIED_FILES" "$$UNTRACKED_FILES" | tr ' ' '\n' | sort -u); \
		if [ -n "$$FILES" ]; then \
			if type -p parallel >/dev/null; then \
				parallel -u --will-cite --bar clang-format -i ::: $$FILES; \
			else \
				echo "$$FILES" | xargs -P$$(nproc) -n1 clang-format -i; \
			fi; \
		fi; \
		popd >/dev/null; \
	else \
		echo -e "$(INDENT_PRINT)$(TRED)--- Cannot format code$(TNORMAL) (clang-format not found)"; \
    exit 1; \
  fi

.PHONY: format-all
format-all:
	$(SILENT) echo -e "$(INDENT_PRINT)[FMT] Formatting all local files (git)"
	$(SILENTSYMB)if type -p clang-format >/dev/null; then \
		pushd $(SRCDIR) >/dev/null; \
		FILES=$$(git ls-files \*.{h,cpp}); \
		if [ -n "$$FILES" ]; then \
			if type -p parallel >/dev/null; then \
				parallel -u --will-cite --bar clang-format -i ::: $$FILES; \
			else \
				echo "$$FILES" | xargs -P$$(nproc) -n1 clang-format -i; \
			fi; \
		fi; \
		popd >/dev/null; \
	else \
		echo -e "$(INDENT_PRINT)$(TRED)--- Cannot format code$(TNORMAL) (clang-format not found)"; \
    exit 1; \
  fi

.PHONY: format-diff
format-diff:
	$(SILENTSYMB)if type -p clang-format >/dev/null; then \
		pushd $(SRCDIR) >/dev/null; \
		FILES=$$(git ls-files \*.{h,cpp}); \
		if [ -n "$$FILES" ]; then \
			for f in $$FILES; do \
				DIFF=$$(diff -u $$f <(clang-format $$f)); \
				if [ -n "$$DIFF" ]; then \
					echo -e "$(INDENT_PRINT)$(TRED)--- Formatting errors in $$f$(TNORMAL)"; \
					cat <<< $$DIFF; \
				fi; \
			done; \
		fi; \
		popd >/dev/null; \
	else \
		echo -e "$(INDENT_PRINT)$(TRED)--- Cannot format code$(TNORMAL) (clang-format not found)"; \
    exit 1; \
  fi

# The local version only updates modified files by default
.PHONY: format
format: format-modified-clang
