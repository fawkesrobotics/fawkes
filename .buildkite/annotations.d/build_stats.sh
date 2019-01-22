#!/usr/bin/env bash

############################################################################
#  Annotate build with build stats
#
#  Created: Tue Jan 22 10:11:59 2019 +0100
#  Copyright  2018-2019  Tim Niemueller [www.niemueller.org]
############################################################################

#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU Library General Public License for more details.
#
#  Read the full text in the LICENSE.GPL file in the doc directory.

SCRIPT_PATH=$(dirname $(readlink -f ${BASH_SOURCE[0]}))

# Source standard environment, may setup ccache if available
source /etc/profile

source $SCRIPT_PATH/../functions.sh

# Error out on any error in the script, pipes etc.
set -euo pipefail

STATSFILE=$(realpath $SCRIPT_PATH/../../buildstats.db)

LABEL=$(emoji_html_img "${BUILDKITE_LABEL:-:question: Unknown System}")

if [ ! -f $STATSFILE ]; then
	echo No build stats have been generated on $LABEL
else
	LIBS_built=$(sqlite3 $STATSFILE \
	             'SELECT count(*) FROM buildstats WHERE type="lib" AND state="built"'); \
	LIBS_total=$(sqlite3 $STATSFILE \
	             'SELECT count(*) FROM buildstats WHERE type="lib"'); \
	LIBS_ratio=$(( $LIBS_built * 100 / $LIBS_total )); \
	LIBS_color=$(if [ -n "${BUILD_STATS_NO_COLOR:-}" ]; then echo "text-muted"; \
	             elif [ $LIBS_ratio -ge 90 ]; then echo "text-success"; \
	             elif [ $LIBS_ratio -ge 80 ]; then echo -e "text-warning"; \
	             else echo -e "text-danger"; fi)
	LIBS_skipped=$(sqlite3 $STATSFILE 'SELECT item FROM buildstats WHERE type="lib" AND state="skipped"')

	PLUGINS_built=$(sqlite3 $STATSFILE \
	                'SELECT count(*) FROM buildstats WHERE type="plugin" AND state="built"'); \
	PLUGINS_total=$(sqlite3 $STATSFILE \
	                'SELECT count(*) FROM buildstats WHERE type="plugin"'); \
	PLUGINS_ratio=$(( $PLUGINS_built * 100 / $PLUGINS_total )); \
	PLUGINS_color=$(if [ -n "${BUILD_STATS_NO_COLOR:-}" ]; then echo "text-muted"; \
	                elif [ $PLUGINS_ratio -ge 90 ]; then echo "text-success"; \
	                elif [ $PLUGINS_ratio -ge 80 ]; then echo -e "text-warning"; \
	                else echo -e "text-danger"; fi)
	PLUGINS_skipped=$(sqlite3 $STATSFILE 'SELECT item FROM buildstats WHERE type="plugin" AND state="skipped"')

	BINS_built=$(sqlite3 $STATSFILE \
	             'SELECT count(*) FROM buildstats WHERE type="bin" AND state="built"'); \
	BINS_total=$(sqlite3 $STATSFILE \
	             'SELECT count(*) FROM buildstats WHERE type="bin"'); \
	BINS_ratio=$(( $BINS_built * 100 / $BINS_total )); \
	BINS_color=$(if [ -n "${BUILD_STATS_NO_COLOR:-}" ]; then echo "text-muted"; \
	             elif [ $BINS_ratio -ge 90 ]; then echo "text-success"; \
	             elif [ $BINS_ratio -ge 80 ]; then echo -e "text-warning"; \
	             else echo -e "text-danger"; fi)
	BINS_skipped=$(sqlite3 $STATSFILE 'SELECT item FROM buildstats WHERE type="bin" AND state="skipped"')

	cat <<-EOM
	<h4>$LABEL Build Coverage</h4>
	<div class="flex">
	  <div class="p1 flex-auto">
	    <div class="my1 h4 text-info"><strong>Libraries</strong></div>
	    <div class="">
	      <span class="$LIBS_color"><big><big>$LIBS_ratio</big></big>%</span><br>
	      Built $LIBS_built of $LIBS_total
	    </div>
	EOM
	if (( $LIBS_total - $LIBS_built > 0 )); then
		cat <<-EOM
		    <details class="mt1">
		      <summary>Skipped Libraries</summary>
		      <ul>
		$(for c in $LIBS_skipped; do echo "        <li>$c</li>"; done)
		      </ul>
		    </details>
		EOM
	fi
	cat <<-EOM
	  </div>
	  <div class="p1 flex-auto">
	    <div class="my1 h4 text-info"><strong>Plugins</strong></div>
	    <div class="">
	      <span class="$PLUGINS_color"><big><big>$PLUGINS_ratio</big></big>%</span><br>
	      Built $PLUGINS_built of $PLUGINS_total
	    </div>
	EOM
	if (( $PLUGINS_total - $PLUGINS_built > 0 )); then
		cat <<-EOM
		    <details class="mt1">
		      <summary>Skipped Plugins</summary>
		      <ul>
		$(for c in $PLUGINS_skipped; do echo "        <li>$c</li>"; done)
		      </ul>
		    </details>
		EOM
	fi
	cat <<-EOM
	  </div>
	  <div class="p1 flex-auto">
	    <div class="my1 h4 text-info"><strong>Executables</strong></div>
	    <div class="">
	      <span class="$BINS_color"><big><big>$BINS_ratio</big></big>%</span><br>
	      Built $BINS_built of $BINS_total
	    </div>
	EOM
	if (( $BINS_total - $BINS_built > 0 )); then
		cat <<-EOM
		    <details class="mt1">
		      <summary>Skipped Executables</summary>
		      <ul>
		$(for c in $BINS_skipped; do echo "        <li>$c</li>"; done)
		      </ul>
		    </details>
		EOM
	fi

	echo "  </div>"
	echo "</div>"
fi
