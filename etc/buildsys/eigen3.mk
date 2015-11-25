#*****************************************************************************
#                 Makefile Build System for Fawkes: Eigen3 bits
#                            -------------------
#   Created on Sun Jul 13 16:09:54 2014
#   Copyright (C) 2011-2014 by Tim Niemueller, Carologistics RoboCup Team
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
$(error config.mk must be included before pcl.mk)
endif

ifndef __buildsys_eigen3_mk_
__buildsys_eigen3_mk_ := 1

ifneq ($(PKGCONFIG),)
  HAVE_EIGEN3 = $(if $(shell $(PKGCONFIG) --exists 'eigen3'; echo $${?/1/}),1,0)
endif

ifeq ($(HAVE_EIGEN3),1)
  CFLAGS_EIGEN3  = -DHAVE_EIGEN3 $(subst -I,-isystem,$(shell $(PKGCONFIG) --cflags 'eigen3')) \
		   -DEIGEN_USE_NEW_STDVECTOR \
		   -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
  LDFLAGS_EIGEN3 = $(shell $(PKGCONFIG) --libs 'eigen3')
  ifeq ($(CC),clang)
    ifeq ($(call clang_atleast_version,3,4),1)
      CFLAGS_EIGEN3 += -Wno-deprecated-register
    endif
  endif
  ifeq ($(CC),gcc)
    ifeq ($(call gcc_atleast_version,5,0),1)
      CFLAGS_EIGEN3 += -Wno-deprecated-declarations
    endif
  endif
endif

endif # __buildsys_eigen3_mk_
