
/***************************************************************************
 *  msl2008.h - Fawkes mid-size refbox 2008 protocol repeater
 *
 *  Created: Wed Apr 01 10:36:08 2009
 *  Copyright  2009  Stefan Schiffer [stefanschiffer.de]
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

#ifndef __TOOLS_REFBOXREP_MSL2008_H_
#define __TOOLS_REFBOXREP_MSL2008_H_

#include "refbox_state_sender.h"

#include <cstdlib>
#include <string>
#include <vector>

namespace fawkes {
  class MulticastDatagramSocket;
}

namespace xmlpp {
  class DomParser;
  class Node;
}

class Msl2008RefBoxRepeater
{
 public:
  Msl2008RefBoxRepeater(RefBoxStateSender &rss,
			const char *refbox_host, unsigned short int refbox_port);
  ~Msl2008RefBoxRepeater();

  void run();

 private:
  void process_string(char *buf, size_t len);
  void reconnect();

 private:
  RefBoxStateSender    &__rss;
  fawkes::MulticastDatagramSocket *__s;

  unsigned int __score_cyan;
  unsigned int __score_magenta;

  bool __quit;

  char *__refbox_host;
  unsigned short int __refbox_port;


  xmlpp::DomParser *dom;
  xmlpp::Node      *root;

};

#endif
