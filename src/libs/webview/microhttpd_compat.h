/*
 * microhttpd_compat.h
 * Copyright (C) 2020 Tarik Viehmann <viehmann@kbsg.rwth-aachen.de>
 *
 * Distributed under terms of the MIT license.
 */

#ifndef MICROHTTPD_COMPAT_H
#define MICROHTTPD_COMPAT_H
#include <microhttpd.h>
#if MHD_VERSION >= 0x00097002
#	define MHD_RESULT MHD_Result
#else
#	define MHD_RESULT int
#endif

#endif /* !MICROHTTPD_COMPAT_H */
