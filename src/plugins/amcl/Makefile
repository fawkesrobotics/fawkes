#*****************************************************************************
#     Makefile Build System for Fawkes: BlackBoard Synchronization Plugin
#                            -------------------
#   Created on Wed May 16 16:00:45 2012
#   copyright (C) 2006-2012 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

BASEDIR = ../../..
include $(BASEDIR)/etc/buildsys/config.mk

LIBS_libfawkes_amcl_pf = m
OBJS_libfawkes_amcl_pf = pf/pf.o pf/pf_kdtree.o pf/pf_pdf.o pf/pf_vector.o \
			 pf/eig3.o pf/pf_draw.o

LIBS_libfawkes_amcl_map = m
OBJS_libfawkes_amcl_map = map/map.o map/map_cspace.o map/map_range.o \
			  map/map_store.o map/map_draw.o

LIBS_libfawkes_amcl_sensors = m fawkes_amcl_pf fawkes_amcl_map
OBJS_libfawkes_amcl_sensors = sensors/amcl_sensor.o sensors/amcl_odom.o \
			      sensors/amcl_laser.o

LIBS_amcl = fawkescore fawkesutils fawkesaspects fawkesinterface \
	    fawkesblackboard fvutils \
	    fawkes_amcl_pf fawkes_amcl_map fawkes_amcl_sensors
OBJS_amcl = amcl_plugin.o amcl_thread.o

OBJS_all    = $(OBJS_amcl) $(OBJS_libfawkes_amcl_pf) \
	      $(OBJS_libfawkes_amcl_map) $(OBJS_libfawkes_amcl_sensors)
PLUGINS_all = $(PLUGINDIR)/amcl.$(SOEXT)
LIBS_all    = $(LIBDIR)/libfawkes_amcl_pf.$(SOEXT) \
	      $(LIBDIR)/libfawkes_amcl_map.$(SOEXT) \
	      $(LIBDIR)/libfawkes_amcl_sensors.$(SOEXT)

# Ordering constraints
$(LIBDIR)/libfawkes_amcl_sensors.$(SOEXT): | $(LIBDIR)/libfawkes_amcl_pf.$(SOEXT) $(LIBDIR)/libfawkes_amcl_map.$(SOEXT)
$(PLUGINS_all): | $(LIBDIR)/libfawkes_amcl_sensors.$(SOEXT) $(LIBDIR)/libfawkes_amcl_pf.$(SOEXT) $(LIBDIR)/libfawkes_amcl_map.$(SOEXT)

include $(BUILDSYSDIR)/base.mk

