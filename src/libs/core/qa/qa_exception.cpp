
/***************************************************************************
 *  example_exception.cpp - Example for using exceptions
 *
 *  Generated: Sun Sep 17 14:00:26 2006 (German Medical Library)
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

// Do not mention in API doc
/// @cond EXAMPLES

#include <core/exception.h>

#include <stdlib.h>
#include <iostream>
#include <cstdarg>

using namespace fawkes;

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
  if ( r < (RAND_MAX / 2)) {
    throw ExampleSmallException();
  } else if ( r > (RAND_MAX - RAND_MAX / 20)) {
    //printf("Throwing boom\n");
    //throw ExampleUnhandledException();
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
  va_list va;
  va_start(va, format);
  throw Exception(format, va);
  va_end(va);
  /*
  throw Exception("Format received: %s", format);
  */
}

int
main(int argc, char **argv)
{
  srand(42);

  // errno exception
  // throw Exception(1, "test %i %s", 3, "blub");

  // throw variadic exception
  // variadic_func("test %i %s %i %f", 4, "haha", 4, 3.2);

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
