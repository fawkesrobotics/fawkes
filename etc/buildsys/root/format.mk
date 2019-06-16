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
format-clang: check-parallel
	$(SILENT) echo -e "$(INDENT_PRINT)[FMT] Formatting full tree"
	$(SILENTSYMB)if type -p clang-format >/dev/null; then \
		ALL_FILES=$$(git ls-files \*.{h,cpp}); \
		if type -p parallel >/dev/null; then \
			parallel -u --will-cite --bar clang-format -i ::: $$ALL_FILES; \
		else \
			echo "$$ALL_FILES" | xargs -P$$(nproc) -n1 clang-format -i; \
		fi; \
	else \
		echo -e "$(INDENT_PRINT)$(TRED)--- Cannot format code$(TNORMAL) (clang-format not found)"; \
    exit 1; \
  fi

.PHONY: format-modified-clang
format-modified-clang: check-parallel
	$(SILENT) echo -e "$(INDENT_PRINT)[FMT] Formatting modified files (git)"
	$(SILENTSYMB)if type -p clang-format >/dev/null; then \
		ALL_FILES=$$(git ls-files -m \*.{h,cpp}); \
		if type -p parallel >/dev/null; then \
			parallel -u --will-cite --bar clang-format -i ::: $$ALL_FILES; \
		else \
			echo "$$ALL_FILES" | xargs -P$$(nproc) -n1 clang-format -i; \
		fi; \
	else \
		echo -e "$(INDENT_PRINT)$(TRED)--- Cannot format code$(TNORMAL) (clang-format not found)"; \
    exit 1; \
  fi

.PHONY: format-branch-clang
format-branch-clang: check-parallel
	$(SILENT) echo -e "$(INDENT_PRINT)[FMT] Formatting branch (git)"
	$(SILENT) git fetch --quiet origin master
	$(SILENT) \
	MERGE_BASE=$$(diff --old-line-format='' --new-line-format='' <(git rev-list --first-parent \
	              "origin/master") <(git rev-list --first-parent "HEAD") | head -1); \
	MERGE_BASE_SHORT=$$(git rev-parse --short $$MERGE_BASE); \
	HEAD_HASH=$$(git rev-parse --short HEAD); \
	echo -e "$(INDENT_PRINT)[FMT] Processing range $$MERGE_BASE_SHORT..$$HEAD_HASH"; \
	if type -p clang-format >/dev/null; then \
		MERGE_BASE=$$MERGE_BASE HEAD_HASH=$$HEAD_HASH git filter-branch -f --tree-filter 'PREV=$$(map $$(git rev-parse $$GIT_COMMIT^)); git show $$HEAD_HASH:etc/format-scripts/format-tree-filter.sh > etc/format-scripts/.format-tree-filter-outoftree.sh; chmod +x etc/format-scripts/.format-tree-filter-outoftree.sh; etc/format-scripts/.format-tree-filter-outoftree.sh $$MERGE_BASE $$GIT_COMMIT $$PREV; rm -f etc/format-scripts/.format-tree-filter-outoftree.sh' -- $$MERGE_BASE..HEAD; \
	else \
		echo -e "$(INDENT_PRINT)$(TRED)--- Cannot format code$(TNORMAL) (clang-format not found)"; \
    exit 1; \
  fi

.PHONY: format-emacs
format-emacs: check-parallel
	$(SILENT) echo -e "$(INDENT_PRINT)[FMT] Formatting (emacs) where necessary"
	$(SILENT) echo -e "$(INDENT_PRINT)[FMT] $(TYELLOW)Use 'format' target to use clang-format$(TNORMAL)"
	$(SILENTSYMB)if type -p emacs >/dev/null; then \
		ALL_FILES=$$(git ls-files \*.{h,cpp}); \
		if type -p parallel >/dev/null; then \
			export FAWKES_BASEDIR=$(abspath $(FAWKES_BASEDIR)); \
			parallel -u --will-cite --bar --results /tmp/emacs-format -- emacs -q --batch {} -l $(abspath $(FAWKES_BASEDIR))/etc/format-scripts/emacs-format-cpp.el -f format-code ::: $$ALL_FILES; \
			rm -rf /tmp/emacs-format; \
		else \
			echo "$$ALL_FILES" | xargs -P$$(nproc) -n1 -I '{}' -- emacs -q --batch {} -l $(abspath $(FAWKES_BASEDIR))/etc/format-scripts/emacs-format-cpp.el -f format-code; \
		fi; \
	else \
		echo -e "$(INDENT_PRINT)$(TRED)--- Cannot format code$(TNORMAL) (emacs not found)"; \
    exit 1; \
  fi

# Using clang-format is the default, faster and better results
.PHONY: format
format: format-clang

# Format currently modified files
.PHONY: format-modified
format-modified: format-modified-clang

# Filter branch fixing the format in each commit of the branch
.PHONY: format-branch
format-branch: format-branch-clang

endif # __buildsys_root_format_mk_
