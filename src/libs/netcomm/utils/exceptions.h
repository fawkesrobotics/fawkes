
/***************************************************************************
 *  exceptions.h - Generic network related exceptions
 *
 *  Created: Wed Nov 14 13:25:38 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __NETCOMM_UTILS_EXCEPTIONS_H_
#define __NETCOMM_UTILS_EXCEPTIONS_H_

#include <core/exception.h>

class ConnectionDiedException : public Exception
{
 public:
  ConnectionDiedException(const char *format, ...);
};

#endif
