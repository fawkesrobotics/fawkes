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
#           $Id:Makefile 419 2007-11-08 09:45:51Z liebenberg $
# last modified: $Date:2007-11-08 10:45:51 +0100 (Do, 08 Nov 2007) $
#            by: $Author:liebenberg $
#

BASEDIR = ../../..

include $(BASEDIR)/etc/buildsys/config.mk
-include $(BASEDIR)/src/plugins/navigator/navigator.mk

ifeq ($(OBJSSUBMAKE),1)
.PHONY: warning_nonavi
warning_nonavi:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting JoystickTool (navigator not built)$(TNORMAL)"
endif

LIBS_joystick = core utils netcomm navigator fawkesmainappmsgs
OBJS_joystick = joystick_tool.o	\
		        cjoystick.o

OBJS_all =	$(OBJS_joystick)

ifeq ($(BUILD_NAVIGATOR),1)
BINS_all = $(BINDIR)/joystick
else
all: warning_nonavi
endif

include $(BASEDIR)/etc/buildsys/base.mk

