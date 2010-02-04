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

.SUFFIXES:

include $(BUILDSYSDIR)/config.mk

MAKECMDGOALS ?= all
MAKETARGET = $(MAKE) --no-print-directory --no-keep-going -C $@ \
                     -f $(CURDIR)/Makefile OBJSSUBMAKE=1 \
                     SRCDIR=$(CURDIR) BASEDIR=../$(BASEDIR) $(MAKECMDGOALS)

.PHONY: $(OBJDIR)
$(OBJDIR):
	+@[ -d $@ ] || mkdir -p $@
	+@$(MAKETARGET) || exit $$?

% :: $(OBJDIR) ; @:

.PHONY: nothing
nothing: ; @:

