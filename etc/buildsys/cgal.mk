#*****************************************************************************
#                 Makefile Build System for Fawkes: CGAL bits
#                            -------------------
#   Created on Tue Apr 07 12:00:34 2015
#   Copyright (C) 2011-2015 by Tim Niemueller, Carologistics RoboCup Team
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
$(error config.mk must be included before cgal.mk)
endif

ifndef __buildsys_cgal_mk_
__buildsys_cgal_mk_ := 1

ifndef __buildsys_boost_mk_
include $(BUILDSYSDIR)/boost.mk
endif

CGAL_REQ_BOOST_LIBS = system thread
CGAL_HAVE_BOOST_LIBS = $(call boost-have-libs,$(REQ_BOOST_LIBS))
CGAL_ERRORS=

ifeq ($(CGAL_HAVE_BOOST_LIBS),1)
  ifneq ($(wildcard $(SYSROOT)/usr/include/CGAL/version.h),)
    ifneq ($(wildcard $(SYSROOT)/usr/include/gmp.h),)
      ifneq ($(wildcard $(SYSROOT)/usr/include/mpfr.h),)
        HAVE_CGAL:=1
        CFLAGS_CGAL:= -DHAVE_CGAL $(call boost-libs-cflags,$(CGAL_REQ_BOOST_LIBS))
        LDFLAGS_CGAL:=-lCGAL -lCGAL_Core -lgmp -lmpfr -lm \
		      $(call boost-libs-ldflags,$(REQ_BOOST_LIBS))
      else
        CGAL_ERRORS += "MPFR_no_found"
      endif
    else
      CGAL_ERRORS += "GMP_no_found"
    endif
  else
    CGAL_ERRORS += "CGAL_no_found"
  endif
else
  ifneq ($(HAVE_BOOST_LIBS),1)
    CGAL_ERROS += = $(foreach l,$(REQ_BOOST_LIBS),$(if $(call boost-have-lib,$l),, Boost_library_$l_not_found))
  endif
endif

ifneq ($(CGAL_ERROS),)
  _CGAL_COMMA := ,
  CGAL_ERROR=$(subst _, ,$(subst :,$(_CGAL_COMMA) ,$(call merge,:,$(CGAL_ERRORS))))
endif

endif # __buildsys_cgal_mk_
