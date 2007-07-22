#*****************************************************************************
#             Makefile Build System for Fawkes: Navigator Plugin
#                            -------------------
#   Created on Sun Jul 22 16:14:56 2007
#   copyright (C) 2006-2007 by Tim Niemueller, AllemaniACs RoboCup Team
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

# Check for GTS (Fedora packages gts and gts-devel)
ifneq ($(PKGCONFIG),)
  HAVE_GTS = $(if $(shell $(PKGCONFIG) --exists 'gts'; echo $${?/1/}),1,0)
endif

# check for VMC (AllemaniACs custom packages vmc and vmc-devel)
ifneq ($(realpath /usr/include/vmc/LayerClasses/VMC_API.h),)
  HAVE_VMC = 1
endif

BUILD_NAVIGATOR = 1
ifneq ($(HAVE_VMC),1)
  BUILD_NAVIGATOR = 0
endif
ifneq ($(HAVE_GTS),1)
  BUILD_NAVIGATOR = 0
endif

