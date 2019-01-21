#*****************************************************************************
#          Makefile Build System for Fawkes: per-target build stats
#                            -------------------
#   Created on Sat Jan 19 19:39:45 2019 +0100
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
$(error config.mk must be included before stats.mk)
endif

ifndef __buildsys_stats_mk_
__buildsys_stats_mk_ := 1

SQLITE = sqlite3 -batch -init <(echo .timeout 5000)

.PHONY: stats stats-gui stats-test

ifneq ($(BUILDSTATS),)

  ifeq ($(shell type -p sqlite3),)
stats:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Cannot generate build stats $(TNORMAL) (sqlite3 not found)"

  else

stats_sql_add = "INSERT OR REPLACE INTO buildstats (type, item, state) VALUES ('%s', '%s', '%s')"

STATS_bins = $(BINS_build) $(if $(filter gui,$(MAKECMDGOALS)),$(BINS_gui)) $(if $(filter test,$(MAKECMDGOALS)),$(BINS_test))
STATS_bin_built   = $(patsubst $(BINDIR)/%,%,$(filter $(STATS_bins),$(BINS_all)))
STATS_bin_skipped = $(patsubst $(BINDIR)/%,%,$(filter-out  $(STATS_bins),$(BINS_all)))

STATS_libs = $(LIBS_build) $(if $(filter gui,$(MAKECMDGOALS)),$(LIBS_gui)) $(if $(filter test,$(MAKECMDGOALS)),$(LIBS_test))
STATS_lib_built   = $(patsubst $(LIBDIR)/%,%,$(filter $(STATS_libs),$(LIBS_all)))
STATS_lib_skipped = $(patsubst $(LIBDIR)/%,%,$(filter-out $(LIBS_build) $(LIBS_gui) $(LIBS_test),$(LIBS_all)))

STATS_plugins = $(PLUGINS_build) $(if $(filter gui,$(MAKECMDGOALS)),$(PLUGINS_gui)) $(if $(filter test,$(MAKECMDGOALS)),$(PLUGINS_test))
STATS_plugin_built   = $(patsubst $(PLUGINDIR)/%,%,$(filter $(STATS_plugins),$(PLUGINS_all)))
STATS_plugin_skipped = $(patsubst $(PLUGINDIR)/%,%,$(filter-out $(PLUGINS_build) $(PLUGINS_gui) $(PLUGINS_test),$(PLUGINS_all)))

# The following stats target adds entries to the database.
# Note that it is three nested loops, the outer two (foreach) are make
# constructs, while the inner (for) is the bash script. The outer iterates
# over type and states, the inner iterates over items. This is done
# to minimize code repetition.
# In the sqlite3 command, note that $t and $s are variables resolved
# by make, while $$c is resolved to bash's loop variable ("for c").

stats: | presubdirs $(BINS_build) $(LIBS_build) $(PLUGINS_build) $(MANPAGES_build) $(TARGETS_all) $(EXTRA_all)
	$(SILENT) \
	$(foreach t,bin lib plugin,$(foreach s,built skipped,$(if $(strip $(STATS_$t_$s)), \
	if [ -e "$(STATSFILE)" ]; then \
		for c in $(STATS_$t_$s); do \
			$(SQLITE) $(STATSFILE) "$$(printf $(stats_sql_add) $t $$c $s)"; \
		done; \
	fi; \
	)))

stats-gui: stats $(BINS_gui) $(LIBS_gui) $(PLUGINS_gui) $(MANPAGES_gui) $(TARGETS_gui)
stats-test: stats $(BINS_test) $(LIBS_test) $(PLUGINS_test) $(TARGETS_test)

endif
else

stats:
stats-gui:
stats-test:

endif

endif # __buildsys_stats_mk_
