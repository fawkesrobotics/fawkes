#*****************************************************************************
#                      Makefile Build System for Fawkes
#                            -------------------
#   Created on Sun Sep 03 14:14:14 2006
#   copyright (C) 2006 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************
#
#           $Id$
# last modified: $Date$
#            by: $Author$
#
#*****************************************************************************

.DEFAULT:
#.DEFAULT: all

include $(BASEDIR)/etc/buildsys/config.mk
ifneq ($(OBJDIR),$(notdir $(CURDIR)))
  ifneq (clean,$(MAKECMDGOALS))
    include $(BASEDIR)/etc/buildsys/objsdir.mk
  else
    include $(BASEDIR)/etc/buildsys/rules.mk
  endif
else
  include $(BASEDIR)/etc/buildsys/rules.mk
endif

