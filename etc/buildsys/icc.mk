#*****************************************************************************
#           Makefile Build System for Fawkes: Intel Compiler Settings
#                            -------------------
#   Created on Sun Sep 03 14:14:14 2006
#   Copyright (C) 2006-2007 by Tim Niemueller, AllemaniACs RoboCup Team
#
##*****************************************************************************
#
##   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   #   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#   #
#*****************************************************************************

# Use Intel compiler, only icc >= 10.1 is supported

# We ignore several warnings that are not important to use (-wd flag):
# 177: entity-kind "entity" was declared but never referenced (like catch Exception, if just re-thrown or unused)
# 383: value copied to temporary, reference to temporary used
# 810: conversion from "type" to "type" may lose significant bits (like htons and double->float)
# 869: entity-kind "entity" was never referenced (unused parameters)
# 981: operands are evaluated in unspecified order (some string operations)
# 1418: external function definition with no prior declaration
# 1572: floating-point equality and inequality comparisons are unreliable

ICC_PATH = $(wildcard /opt/intel/Compiler/*/*)
CC = $(ICC_PATH)/bin/ia32/icc

DEFAULT_INCLUDES += -I$(ICC_PATH)/include
LDFLAGS_LIBDIRS  += -Wl,-R$(ICC_PATH)/lib
CFLAGS_BASE      += -wd177,383,810,869,981,1418,1572
LDFLAGS_BASE     += -L$(ICC_PATH)/lib -lguide -lpthread
CFLAGS_OPENMP     = -openmp
LDFLAGS_OPENMP    = -lgomp

