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

include $(BUILDCONFDIR)/navgraph/navgraph.mk
include $(BUILDSYSDIR)/boost.mk

CGAL_REQ_BOOST_LIBS = system thread
CGAL_HAVE_BOOST_LIBS = $(call boost-have-libs,$(REQ_BOOST_LIBS))

ifeq ($(HAVE_NAVGRAPH)$(CGAL_HAVE_BOOST_LIBS),11)
  ifneq ($(wildcard $(SYSROOT)/usr/include/CGAL/version.h),)
    ifneq ($(wildcard $(SYSROOT)/usr/include/gmp.h),)
      ifneq ($(wildcard $(SYSROOT)/usr/include/mpfr.h),)
        HAVE_CGAL:=1
        CFLAGS_CGAL:= -DHAVE_CGAL $(call boost-libs-cflags,$(CGAL_REQ_BOOST_LIBS))
        LDFLAGS_CGAL:=-lCGAL -lCGAL_Core -lgmp -lmpfr -lm \
		      $(call boost-libs-ldflags,$(REQ_BOOST_LIBS))
      else
        WARN_TARGETS += warning_mpfr
      endif
    else
      WARN_TARGETS += warning_gmp
    endif
  else
    WARN_TARGETS += warning_cgal
  endif
else
  ifneq ($(HAVE_NAVGRAPH),1)
    WARN_TARGETS += warning_navgraph
  endif
  ifneq ($(HAVE_BOOST_LIBS),1)
    WARN_TARGETS_BOOST = $(foreach l,$(REQ_BOOST_LIBS),$(if $(call boost-have-lib,$l),, warning_boost_$l))
  endif
endif


ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)  $(WARN_TARGETS_BOOST)
.PHONY: $(WARN_TARGETS)
warning_mpfr:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)navgraph generators library not available$(TNORMAL) (GMP not found)"
warning_gmp:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)navgraph generators library not available$(TNORMAL) (MPFR not found)"
warning_cgal:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)navgraph generators library not available$(TNORMAL) (CGAL not found)"
$(WARN_TARGETS_BOOST): warning_boost_%:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)navgraph generators library not available$(TNORMAL) (Boost library $* not found)"
endif
