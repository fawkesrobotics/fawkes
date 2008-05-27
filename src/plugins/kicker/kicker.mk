#*****************************************************************************
#           Makefile Build System for Fawkes: Kicker Plugin Config
#                            -------------------
#   Created on Fri May 18 15:34:42 2007
#   Copyright (C) 2007 by Daniel Beck, AllemaniACs RoboCup Team
#   Copyright (C) 2007 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

# We now assume a system wide installation via RPM
#IOWKIT_SDK=/opt/iowkit
#KICKER_INCDIRS += $(IOWKIT_SDK)/include
#KICKER_LIBDIRS += $(IOWKIT_SDK)/lib

# Check whether IowKit is installed
ifneq ($(wildcard /usr/include/iowkit.h),)
  HAVE_IOWKIT = 1
endif

