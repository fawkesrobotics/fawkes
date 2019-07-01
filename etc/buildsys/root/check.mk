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
ifndef __buildsys_root_yamllint_mk_
$(error yamllint.mk must be included before check.mk)
endif

ifndef __buildsys_root_check_mk_
__buildsys_root_check_mk_ := 1

YAMLLINT ?= 1

GITFILES = git ls-files --full-name \*.{h,cpp} | awk "{ print \"$$(git rev-parse --show-toplevel)/\"\$$1 }"

.PHONY: license-check
license-check:
	$(SILENT) echo -e "$(INDENT_PRINT)[CHK] Checking license headers"
	$(SILENTSYMB)if which perl >/dev/null; then \
		perl $(FAWKES_BASEDIR)/etc/licscripts/find_invlic.pl -k <($(GITFILES)) -p src -p etc $(if $(SUBMODULE_EXTERN),-p fawkes/src) $(wildcard $(FAWKES_BASEDIR)/doc/headers/lichead*.*); \
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

.PHONY: format-check
format-check: check-parallel
	$(SILENT) echo -e "$(INDENT_PRINT)[CHK] Checking code formatting"
	$(SILENTSYMB)if type -p clang-format >/dev/null; then \
		OFFENDING_FILES=$$($(FAWKES_BASEDIR)/etc/format-scripts/check-all-files.sh); \
		if [ "$$?" != "0" ]; then \
			for f in $$OFFENDING_FILES; do \
				echo -e "$(INDENT_PRINT)$(TRED)--> $$f is not properly formatted$(TNORMAL)"; \
			done; \
			NUM_FILES=$$(git ls-files \*.{h,cpp} | wc -l); \
			BAD_FILES=$$(wc -w <<< "$$OFFENDING_FILES"); \
			echo -e "$(INDENT_PRINT)$(TRED)--> $$BAD_FILES/$$NUM_FILES have bad formatting.$(TNORMAL)"; \
			exit 1; \
		else \
			echo -e "$(INDENT_PRINT)$(TGREEN)--> All source files properly formatted$(TNORMAL)"; \
		fi; \
	else \
		echo -e "$(INDENT_PRINT)$(TRED)--- Cannot do format check$(TNORMAL) (clang-format not found)"; \
    exit 1; \
  fi

.PHONY: format-check-branch
format-check-branch: check-parallel
	$(SILENT) echo -e "$(INDENT_PRINT)[CHK] Checking code formatting in branch and modified files"
	$(SILENTSYMB)if type -p clang-format >/dev/null; then \
		OFFENDING_FILES=$$(SHOW_PROGRESS="$(if $(HIDE_FORMAT_PROGRESS),no,yes)" $(FAWKES_BASEDIR)/etc/format-scripts/check-branch-files.sh); \
		if [ "$$?" != "0" ]; then \
			NUM_FILES=$$(cut -d$$'\n' -f1 <<< "$$OFFENDING_FILES"); \
			OFFENDING_FILES=$$(cut -d$$'\n' -f2- <<< "$$OFFENDING_FILES"); \
			BAD_FILES=$$(wc -w <<< "$$OFFENDING_FILES"); \
			for f in $$OFFENDING_FILES; do \
				echo -e "$(INDENT_PRINT)$(TRED)--> $$f is not properly formatted$(TNORMAL)"; \
			done; \
			echo -e "$(INDENT_PRINT)$(TRED)--> $$BAD_FILES/$$NUM_FILES have bad formatting.$(TNORMAL)"; \
			exit 1; \
		else \
			echo -e "$(INDENT_PRINT)$(TGREEN)--> All source files properly formatted$(TNORMAL)"; \
		fi; \
	else \
		echo -e "$(INDENT_PRINT)$(TRED)--- Cannot do format check$(TNORMAL) (clang-format not found)"; \
    exit 1; \
  fi

.PHONY: check-parallel
check-parallel:
	$(SILENT)if ! type -p parallel >/dev/null; then \
		echo -e "$(INDENT_PRINT)$(TYELLOW)--> Install GNU Parallel for better progress estimates$(TNORMAL)"; \
	fi

.PHONY: check
check: quick-check quickdoc

.PHONY: quick-check
quick-check: format-check-branch $(if $(subst 0,,$(YAMLLINT)),yamllint) license-check
	$(SILENT)touch $(BASEDIR)/.check_stamp

endif # __buildsys_root_check_mk_
