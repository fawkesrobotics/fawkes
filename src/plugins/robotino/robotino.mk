#*****************************************************************************
#            Makefile Build System for Fawkes: Robotino Plugin
#                            -------------------
#   Created on Fri Jan 07 23:44:11 2011
#   Copyright (C) 2006-2011 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

OPENROBOTINOAPI_DIR=/usr/local/OpenRobotinoAPI

ifneq ($(wildcard $(OPENROBOTINOAPI_DIR)/1/include/robotinoapi_version.h),)
  HAVE_OPENROBOTINO=1
  CFLAGS_OPENROBOTINO  = -Wno-reorder -Wno-unused-function -Wno-delete-non-virtual-dtor -I$(OPENROBOTINOAPI_DIR)/1/include -I$(OPENROBOTINOAPI_DIR)/share/include
  LDFLAGS_OPENROBOTINO = -L$(OPENROBOTINOAPI_DIR)/1/lib/linux -L$(OPENROBOTINOAPI_DIR)/share/lib \
			 -lrec_robotino_com -lrec_core_lt
endif
