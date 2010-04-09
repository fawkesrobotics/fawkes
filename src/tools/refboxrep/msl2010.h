
/***************************************************************************
 *  msl2010.h - Fawkes mid-size refbox 2008 protocol repeater
 *
 *  Created: Wed Apr 01 18:41:00 2010
 *  Copyright  2010  Stefan Schiffer [stefanschiffer.de]
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

#ifndef __TOOLS_REFBOXREP_MSL2010_H_
#define __TOOLS_REFBOXREP_MSL2010_H_

#include "refbox_state_sender.h"

#include <cstdlib>
#include <string>
#include <vector>

namespace fawkes {
  class Socket;
  class StreamSocket;
  class MulticastDatagramSocket;
}

namespace xmlpp {
  class DomParser;
  class Node;
}

class Msl2010RefBoxRepeater
{
 public:
  Msl2010RefBoxRepeater(RefBoxStateSender &rss,
			const char *refbox_host, unsigned short int refbox_port,
			const bool use_multicast = true );
  ~Msl2010RefBoxRepeater();

  void run();

 private:
  void process_string(char *buf, size_t len);
  void reconnect();

 private:
  RefBoxStateSender    &__rss;
  fawkes::Socket *__s;
  //fawkes::StreamSocket *__s;
  //fawkes::MulticastDatagramSocket *__s;

  unsigned int __score_cyan;
  unsigned int __score_magenta;

  bool __quit;

  char *__refbox_host;
  unsigned short int __refbox_port;

  bool __use_multicast;

  xmlpp::DomParser *dom;
  xmlpp::Node      *root;

};

#endif
