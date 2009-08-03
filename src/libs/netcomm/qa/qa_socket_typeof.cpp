
/***************************************************************************
 *  qa_socket_typeof.cpp - Fawkes QA for typeof, used in Socket::accept()
 *
 *  Created: Fri Nov 10 10:33:08 2006 (on train to Google, Hamburg)
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
 *
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

/// @cond QA

#include <typeinfo>

#include <stdio.h>

class A
{
 public:
  A()
  {
    printf("A: constructor called\n");
  }

  virtual ~A() {}

  A *
  clone()
  {
    typeof(this) a = new typeof(*this);
    return a;
  }
};


class B : public A
{
 public:
  B()
  {
    printf("B: constructor called\n");
  }

};


int
main(int argc, char **argv)
{
  B b;
  A *a = b.clone();

  printf("Type of a: %s\n", typeid(a).name());

  B *ba;
  if ( (ba = dynamic_cast<B *>(a)) != NULL ) {
    printf("Dynamic cast of a as B successful\n");
  } else {
    printf("Dynamic cast of a as B FAILED\n");
  }

  delete a;
}

/// @endcond
