
/***************************************************************************
 *  example_exception.cpp - Example for using exceptions
 *
 *  Generated: Sun Sep 17 14:00:26 2006 (German Medical Library)
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth floor, Boston, MA 02111-1307, USA.
 */

// Do not mention in API doc
/// @cond EXAMPLES

#include <core/exception.h>

#include <stdlib.h>
#include <iostream>
#include <cstdarg>

class ExampleSmallException : public Exception
{
 public:
  ExampleSmallException() : Exception("Small Exception") {}
};

class ExampleBigException : public Exception
{
 public:
  ExampleBigException() : Exception("Big Exception") {}
};

class ExampleUnhandledException : public Exception
{
 public:
  ExampleUnhandledException() : Exception("Exception not handled") {}
};


void
throw_some_exception()
{
  int r = rand();
  if ( r < (RAND_MAX / 3)) {
    throw ExampleSmallException();
  } else if ( r > 1 - (RAND_MAX / 20)) {
    printf("Throwing boom\n");
    throw ExampleUnhandledException();
  } else {
    throw ExampleBigException();
  }
}

void
indirect_throw_some_exception()
{
  try {
    throw_some_exception();
  } catch (Exception &e) {
    e.append("More info");
    throw;
  }
}

void
variadic_func(const char *format, ...)
{
  /*
  va_list va;
  va_start(va, format);
  throw Exception(format, va);
  va_end(va);
  */
  throw Exception("Format received: %s", format);
}

int
main(int argc, char **argv)
{
  srand(42);

  // errno exception
  // throw Exception(1, "test %s", "blub");

  // throw variadic exception
  variadic_func("test %s %i %f", "haha", 4, 3.2);

  while (1) {
    try {
      indirect_throw_some_exception();
    } catch (ExampleSmallException &se) {
      std::cout << "Message: " <<  se.what() << std::endl;
      std::cout << "Trace:" << std::endl;
      se.print_trace();
    } catch (ExampleBigException &be) {
      std::cout << "Message: " << be.what() << std::endl;
      std::cout << "Trace:" << std::endl;
      be.print_trace();
    }
  }
}


/// @endcond
