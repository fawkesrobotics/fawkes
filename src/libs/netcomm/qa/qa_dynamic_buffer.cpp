
/***************************************************************************
 *  qa_dynamic_buffer.cpp - Fawkes QA DynamicBuffer
 *
 *  Created: Fri Jun  1 16:03:26 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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

#include <netcomm/utils/dynamic_buffer.h>

#include <iostream>
#include <cstring>
#include <cstdio>

using namespace std;
using namespace fawkes;

int
main(int argc, char **argv)
{

  dynamic_list_t dl;
  DynamicBuffer *dw = new DynamicBuffer(&dl);

  for ( unsigned int i = 0; i < 1000; ++i ) {
    dw->append("test", strlen("test"));
  }

  cout << "Added elements, num_elements: " << dw->num_elements()
       << ", buffer_size: " << dw->buffer_size()
       << ", real_buffer_size: " << dw->real_buffer_size() << endl;

  DynamicBuffer *dr = new DynamicBuffer(&dl, dw->buffer(), dw->buffer_size());

  cout << "Read buffer opened, num_elements: " << dr->num_elements()
       << ", buffer_size: " << dr->buffer_size()
       << ", real_buffer_size: " << dr->real_buffer_size() << endl;

  while ( dr->has_next() ) {
    char tmp[1024];
    memset(tmp, 0, sizeof(tmp));
    size_t size;
    void *buf = dr->next(&size);
    strncpy(tmp, (const char *)buf, size);
    printf("Read string (%lu bytes): '%s'\n", (unsigned long int)size, tmp);
  }

  delete dw;
  delete dr;
}

/// @endcond
