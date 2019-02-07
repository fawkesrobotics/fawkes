#*****************************************************************************
#            Makefile Build System for Fawkes: build stats
#                            -------------------
#   Created on Thu Jan 10 11:23:12 2019 +0100
#   Copyright (C) 2006-2019 by Tim Niemueller [www.niemueller.org]
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
$(error config.mk must be included before root.mk)
endif

ifndef __buildsys_root_stats_mk_
__buildsys_root_stats_mk_ := 1

STATS_FIELDS=\
  type TEXT NOT NULL, \
  item TEXT NOT NULL, \
  state TEXT NOT NULL, \
  time DATETIME DEFAULT CURRENT_TIMESTAMP, \
  PRIMARY KEY (type, item)

ifneq ($(BUILDSTATS),)

.PHONY: stats-prep
stats-prep:
	$(eval BUILT_PARTS += $@)
	$(SILENT)rm -f $(STATSFILE)
	$(SILENT)sqlite3 -batch $(STATSFILE) 'CREATE TABLE buildstats ($(STATS_FIELDS))'

src: stats-prep

all: stats-prep stats-print

endif

.PHONY: stats-print
stats-print: $(if $(filter stats-print,$(MAKECMDGOALS)),,| src)
	$(eval BUILT_PARTS += $@)
	$(SILENT)if [ ! -e $(STATSFILE) ]; then \
		echo -e "$(TRED)No build stats file, rebuild with BUILDSTATS=1.$(TNORMAL)"; \
		exit 8; \
	fi
	$(SILENT) :;\
	echo; \
	echo -e "$(TBLUE)################################ BUILD STATS ################################$(TNORMAL)"; \
	LIBS_built=$$(sqlite3 $(STATSFILE) \
                   'SELECT count(*) FROM buildstats WHERE type="lib" AND state="built"'); \
	LIBS_total=$$(sqlite3 $(STATSFILE) \
                   'SELECT count(*) FROM buildstats WHERE type="lib"'); \
	LIBS_ratio=$$(( $$LIBS_built * 100 / $$LIBS_total )); \
	PLUGINS_built=$$(sqlite3 $(STATSFILE) \
                   'SELECT count(*) FROM buildstats WHERE type="plugin" AND state="built"'); \
	PLUGINS_total=$$(sqlite3 $(STATSFILE) \
                   'SELECT count(*) FROM buildstats WHERE type="plugin"'); \
	PLUGINS_ratio=$$(( $$PLUGINS_built * 100 / $$PLUGINS_total )); \
	BINS_built=$$(sqlite3 $(STATSFILE) \
                   'SELECT count(*) FROM buildstats WHERE type="bin" AND state="built"'); \
	BINS_total=$$(sqlite3 $(STATSFILE) \
                   'SELECT count(*) FROM buildstats WHERE type="bin"'); \
	BINS_ratio=$$(( $$BINS_built * 100 / $$BINS_total )); \
	echo -e "$(TYELLOW)LIBRARIES$(TNORMAL)                  $(TYELLOW)PLUGINS$(TNORMAL)                    $(TYELLOW)EXECUTABLES$(TNORMAL)"; \
	printf "$(TBLUE)Built:$(TNORMAL) %4d/$(TBOLDGRAY)%-4d$(TNORMAL) (%s%3d%%$(TNORMAL))    $(TBLUE)Built:$(TNORMAL) %4d/$(TBOLDGRAY)%-4d$(TNORMAL) (%s%3d%%$(TNORMAL))    $(TBLUE)Built:$(TNORMAL) %4d/$(TBOLDGRAY)%-4d$(TNORMAL) (%s%3d%%$(TNORMAL))" \
	  $$LIBS_built $$LIBS_total \
		$$(if [ $$LIBS_ratio -ge 90 ]; then echo -e "$(TGREEN)"; \
	     elif [ $$LIBS_ratio -ge 80 ]; then echo -e "$(TYELLOW)"; else echo -e "$(TRED)"; fi) \
	  $$LIBS_ratio \
    $$PLUGINS_built $$PLUGINS_total  \
		$$(if [ $$PLUGINS_ratio -ge 90 ]; then echo -e "$(TGREEN)"; \
	     elif [ $$PLUGINS_ratio -ge 80 ]; then echo -e "$(TYELLOW)"; else echo -e "$(TRED)"; fi) \
	  $$PLUGINS_ratio \
    $$BINS_built $$BINS_total \
		$$(if [ $$BINS_ratio -ge 90 ]; then echo -e "$(TGREEN)"; \
	     elif [ $$BINS_ratio -ge 80 ]; then echo -e "$(TYELLOW)"; else echo -e "$(TRED)"; fi) \
	  $$BINS_ratio; \
	echo; \
	if [ $$(expr $$LIBS_total - $$LIBS_built) -gt 0 ]; then \
		echo; \
		LIBS_skipped_items=$$(sqlite3 $(STATSFILE) 'SELECT item FROM buildstats WHERE type="lib" AND state="skipped"'); \
		echo -e "$(TBROWN)Skipped libraries:$(TNORMAL)"; \
		for i in $$LIBS_skipped_items; do \
			echo "  - $$i"; \
		done; \
	fi; \
	if [ $$(expr $$PLUGINS_total - $$PLUGINS_built) -gt 0 ]; then \
		echo; \
		PLUGINS_skipped_items=$$(sqlite3 $(STATSFILE) 'SELECT item FROM buildstats WHERE type="plugin" AND state="skipped"' | sed -e 's/^\(.*\)\.so$$/\1/g'); \
		echo -e "$(TBROWN)Skipped plugins:$(TNORMAL)"; \
		for i in $$PLUGINS_skipped_items; do \
			echo "  - $$i"; \
		done; \
	fi; \
	if [ $$(expr $$BINS_total - $$BINS_built) -gt 0 ]; then \
		echo; \
		BINS_skipped_items=$$(sqlite3 $(STATSFILE) 'SELECT item FROM buildstats WHERE type="bin" AND state="skipped"'); \
		echo -e "$(TBROWN)Skipped executables:$(TNORMAL)"; \
		for i in $$BINS_skipped_items; do \
			echo "  - $$i"; \
		done; \
	fi; \
	echo -e "$(TBLUE)#############################################################################$(TNORMAL)"; \
	echo

endif # __buildsys_root_stats_mk_

