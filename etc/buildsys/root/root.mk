#*****************************************************************************
#     Makefile Build System for Fawkes: Documentation config and targets
#                            -------------------
#   Created on Wed Mar 31 15:20:01 2010
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

ifndef __buildsys_config_mk_
$(error config.mk must be included before root.mk)
endif

ifndef __buildsys_root_root_mk_
__buildsys_parts_root_mk_ := 1

include $(BUILDSYSDIR)/root/docs.mk
include $(BUILDSYSDIR)/root/check.mk
include $(BUILDSYSDIR)/root/btmgmt.mk
include $(BUILDSYSDIR)/root/parallel.mk
include $(BUILDSYSDIR)/root/uncolored.mk

endif # __buildsys_root_root_mk_

