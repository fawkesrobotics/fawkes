#*****************************************************************************
#            Makefile Build System for Fawkes : Joystick Control
#                            -------------------
#   Created on Sat Jun 02 17:38:55 2007
#   copyright (C) 2007 by Martin Liebenberg, AllemaniACs RoboCup Team
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

BASEDIR = ../../..

CFLAGS = -g

LIBS_joystick_control = core utils netcomm navigator config
OBJS_joystick_control = joystick_tool.o	\
	      							  joystick.o

OBJS_all =	$(OBJS_joystick_control)
BINS_all =	$(BINDIR)/joystick_control

include $(BASEDIR)/etc/buildsys/base.mk

