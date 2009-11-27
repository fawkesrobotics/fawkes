#*****************************************************************************
#                      Makefile Build System for Fawkes
#                            -------------------
#   Created on Sun Sep 03 14:14:14 2006
#   Copyright (C) 2006-2007 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

.DEFAULT:

# set here to allow Makefiles w/o included config.mk to work
BUILDSYSDIR  ?= $(abspath $(BASEDIR)/etc/buildsys)

include $(BUILDSYSDIR)/config.mk
ifneq ($(OBJDIR),$(notdir $(CURDIR)))
  ifneq (clean,$(MAKECMDGOALS))
    include $(BUILDSYSDIR)/objsdir.mk
  else
    include $(BUILDSYSDIR)/rules.mk
  endif
else
  include $(BUILDSYSDIR)/rules.mk
endif

