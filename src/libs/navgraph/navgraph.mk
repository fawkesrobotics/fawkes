#*****************************************************************************
#        Makefile Build System for Fawkes: NavGraph Library
#                            -------------------
#   Created on Tue Jan 13 10:47:38 2015
#   Copyright (C) 2015 by Tim Niemueller, AllemaniACs RoboCup Team
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

ifndef __buildsys_config_mk_
$(error config.mk must be included before navgraph.mk)
endif

include $(BUILDSYSDIR)/eigen3.mk

ifndef __navgraph_mk_
__navgraph_mk_ := 1

include $(BUILDSYSDIR)/ext/gmsl

ifneq ($(PKGCONFIG),)
  HAVE_YAMLCPP   = $(if $(shell $(PKGCONFIG) --exists 'yaml-cpp'; echo $${?/1/}),1,0)
endif
ifeq ($(HAVE_YAMLCPP),1)
  HAVE_YAMLCPP_0_2_6 = $(if $(shell $(PKGCONFIG) --atleast-version='0.2.6' 'yaml-cpp'; echo $${?/1/}),1,0)

  CFLAGS_YAMLCPP  = -DHAVE_YAMLCPP $(shell $(PKGCONFIG) --cflags 'yaml-cpp')
  LDFLAGS_YAMLCPP = $(shell $(PKGCONFIG) --libs 'yaml-cpp')
  ifneq ($(HAVE_YAMLCPP_0_2_6),1)
    CFLAGS_YAMLCPP += -DHAVE_OLD_YAMLCPP
  endif

  HAVE_YAMLCPP_0_5 = $(if $(shell $(PKGCONFIG) --atleast-version='0.5' 'yaml-cpp'; echo $${?/1/}),1,0)
  ifeq ($(HAVE_YAMLCPP_0_5),1)
    CFLAGS_YAMLCPP += -DHAVE_YAMLCPP_0_5
  endif
endif

ifeq ($(HAVE_YAMLCPP)$(HAVE_CPP11)$(HAVE_EIGEN3)$(HAVE_CPP11_RANGE_FOR),1111)
  HAVE_NAVGRAPH=1

  CFLAGS_NAVGRAPH  = $(CFLAGS_CPP11)
  LDFLAGS_NAVGRAPH = -lm

else
  ifneq ($(HAVE_YAMLCPP),1)
    NAVGRAPH_ERRORS += "yaml-cpp[-devel]_not_installed"
  endif
  ifneq ($(HAVE_CPP11),1)
    NAVGRAPH_ERRORS += "compiler_does_not_support_C++11"
  endif
  ifneq ($(HAVE_EIGEN3),1)
    NAVGRAPH_ERRORS += "eigen3_not_installed"
  endif
  ifneq ($(HAVE_CPP11_RANGE_FOR),1)
    NAVGRAPH_ERRORS += "GCC_too_old"
  endif

  _NAVGRAPH_COMMA := ,
  NAVGRAPH_ERROR=$(subst _, ,$(subst :,$(_NAVGRAPH_COMMA) ,$(call merge,:,$(NAVGRAPH_ERRORS))))
endif

endif # __navgraph_mk_
