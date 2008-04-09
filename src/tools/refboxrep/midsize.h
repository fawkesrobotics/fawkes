
/***************************************************************************
 *  midsize.h - Fawkes mid-size refbox repeater
 *
 *  Created: Wed Apr 09 10:36:08 2008
 *  Copyright  2008  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#ifndef __TOOLS_REFBOXREP_MIDSIZE_H_
#define __TOOLS_REFBOXREP_MIDSIZE_H_

#include "refbox_state_sender.h"
#include <cstdlib>

class StreamSocket;

class MidsizeRefBoxRepeater
{
 public:
  MidsizeRefBoxRepeater(RefBoxStateSender &rss,
			const char *refbox_host, unsigned short int refbox_port);
  ~MidsizeRefBoxRepeater();

  void run();

 private:
  void process_string(char *buf, size_t len);
  void reconnect();

 private:
  RefBoxStateSender &__rss;
  StreamSocket      *__s;

  unsigned int __score_cyan;
  unsigned int __score_magenta;

  bool __quit;

  char *__refbox_host;
  unsigned short int __refbox_port;
};

#endif
