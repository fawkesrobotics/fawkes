#*****************************************************************************
#                      Makefile Build System for Fawkes
#                            -------------------
#   Created on Thu Mar 13 18:22:59 2014
#   Copyright (C) 2006-2014 by Tim Niemueller, AllemaniACs RoboCup Team
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
$(error config.mk must be included before rules.mk)
endif

ifndef __buildsys_utils_mk_
__buildsys_utils_mk_ := 1

version_number = $(strip $(eval __versplit := $(call split,.,$1))$(call plus,$(call plus,$(call multiply,$(word 1,$(__versplit)),10000),$(call multiply,$(word 2,$(__versplit)),100)),$(if $(word 3,$(__versplit)),$(word 3,$(__versplit)),0)))

endif # __buildsys_utils_mk_
