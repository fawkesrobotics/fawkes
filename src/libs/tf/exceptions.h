/***************************************************************************
 *  exceptions.h - Fawkes tf exceptions
 *
 *  Created: Tue Oct 18 16:38:22 2011
 *  Copyright  2011  Tim Niemueller [www.niemueller.de]
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#ifndef __LIBS_TF_EXCEPTIONS_H_
#define __LIBS_TF_EXCEPTIONS_H_

#include <core/exception.h>

namespace fawkes {
  namespace tf {
#if 0 /* just to make Emacs auto-indent happy */
  }
}
#endif

class TransformException : public fawkes::Exception
{
 public:
  TransformException();
};

class ConnectivityException : public TransformException
{
 public:
  ConnectivityException(const char *format, ...);
};

class LookupException : public TransformException
{
 public:
  LookupException(const char *format, ...);
};

class ExtrapolationException : public TransformException
{
 public:
  ExtrapolationException(const char *format, ...);
};

class InvalidArgumentException : public TransformException
{
 public:
  InvalidArgumentException(const char *format, ...);
};

class DisabledException : public TransformException
{
 public:
  DisabledException(const char *format, ...);
};


} // end namespace tf
} // end namespace fawkes

#endif
