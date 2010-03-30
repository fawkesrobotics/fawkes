#*****************************************************************************
#                      Makefile Build System for Fawkes
#                            -------------------
#   Created on Sun Sep 03 14:14:14 2006
#   Copyright (C) 2006 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

# see http://make.paulandlesley.org/multi-arch.html

ifndef __buildsys_config_mk_
$(error config.mk must be included before objsdir.mk)
endif

.SUFFIXES:

MAKECMDGOALS ?= all
MAKETARGET = $(MAKE) --no-print-directory --no-keep-going -C $@ -f $(CURDIR)/Makefile OBJSSUBMAKE=1 \
             SRCDIR=$(CURDIR) BASEDIR=../$(BASEDIR)  $(filter-out clean,$(MAKECMDGOALS))
MAKELOOP   = for T in $(filter-out clean,$(MAKECMDGOALS)); do $(MAKETARGET) $$T; done
MAKECLEAN  = if [ -d $@ ]; then $(MAKE) --no-print-directory --no-keep-going clean || exit $$?; fi

.PHONY: $(OBJDIR)
$(OBJDIR):
	$(if $(filter clean,$(MAKECMDGOALS)),+@$(MAKECLEAN))
	+@[ -d $@ ] || mkdir -p $@
	$(if $(filter-out clean,$(MAKECMDGOALS)),+@$(MAKETARGET) || exit $$?)

% :: $(OBJDIR) ; @:

.PHONY: nothing
nothing: ; @:

