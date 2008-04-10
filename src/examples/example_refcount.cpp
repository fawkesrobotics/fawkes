
/***************************************************************************
 *  example_refcount.cpp - Example for reference counting
 *
 *  Created: Fri Oct 27 14:09:59 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#include <core/utils/refcount.h>
#include <core/utils/refcounter.h>

#include <iostream>

using namespace std;

class RefCountDerivate : public RefCount
{
public:
  RefCountDerivate()
  {
    cout << "RefCountDerivate::Constructor called" << endl;
  }
  ~RefCountDerivate()
  {
    cout << "RefCountDerivate::Destructor called" << endl;
  }
};


class RefCounterChild
{
public:
  RefCounterChild()
  {
    cout << "RefCounterChild::Constructor called" << endl;
  }
  ~RefCounterChild()
  {
    cout << "RefCounterChild::Destructor called" << endl;
  }

  void test()
  {
    cout << "RefCounterChild::test() called" << endl;
  }
};


int
main(int argc, char **argv)
{
  cout << "Creating a RefCountDerivate instance" << endl;
  RefCountDerivate *rfd = new RefCountDerivate();
  cout << "Referencing three times" << endl;
  rfd->ref();
  rfd->ref();
  rfd->ref();
  cout << "Reference count of rfd is now " << rfd->refcount() << endl;
  cout << "Fully unreferencing" << endl;
  unsigned int refcount = rfd->refcount();
  for (unsigned int i = 0; i < refcount; ++i) {
    rfd->unref();
  }
  cout << "Use valgrind to check that object has been deleted!" << endl;

  cout << "Now using RefCounter with RefCounterChild" << endl;
  RefCounterChild *rfc = new RefCounterChild();
  RefCounter<RefCounterChild> *rc = new RefCounter<RefCounterChild>(rfc);
  cout << "Referencing three times" << endl;
  rc->ref();
  rc->ref();
  rc->ref();
  cout << "Reference count of rc is now " << rc->refcount() << endl;
  cout << "Testing rc's rfc" << endl;
  (*rc)->test();
  cout << "Fully unreferencing" << endl;
  refcount = rc->refcount();
  for (unsigned int i = 0; i < refcount; ++i) {
    rc->unref();
  }
  cout << "Use valgrind to check that object has been deleted!" << endl;

  return 0;
}


/// @endcond
