#*****************************************************************************
#            Makefile Build System for Fawkes: FireVision Config
#                            -------------------
#   Created on Fri May 18 15:34:42 2007
#   copyright (C) 2006-2007 by Daniel Beck, AllemaniACs RoboCup Team
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

IOWKIT_SDK=/opt/iowkit

# Check whether IowKit is installed
ifneq ($(realpath $(IOWKIT_SDK)/include/iowkit.h),)
  ifneq ($realpath $(IOWKIT_SDK)/lib/libiowkit.so),)
    HAVE_IOWKIT = 1
    KICKER_INCDIRS += $(IOWKIT_SDK)/include
    KICKER_LIBDIRS += $(IOWKIT_SDK)/lib
  endif
endif
