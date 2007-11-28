#*****************************************************************************
#             Makefile Build System for Fawkes: Navigator Plugin
#                            -------------------
#   Created on Sun Jul 22 16:14:56 2007
#   Copyright (C) 2006-2007 by Tim Niemueller, AllemaniACs RoboCup Team
#
#   $Id$
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

# Check for GTS (Fedora packages gts and gts-devel)
ifneq ($(PKGCONFIG),)
  HAVE_GTS = $(if $(shell $(PKGCONFIG) --exists 'gts'; echo $${?/1/}),1,0)
endif

# check for VMC (AllemaniACs custom packages vmc and vmc-devel)
ifneq ($(realpath /usr/include/vmc/LayerClasses/CvmcAPI.h),)
  HAVE_VMC = 1
endif

ifeq ($(HAVE_VMC)$(HAVE_GTS),11)
  BUILD_NAVIGATOR = 1
else
  NAVIGATOR_ERROR = (gts[-devel] or vmc[-devel] missing)
endif

