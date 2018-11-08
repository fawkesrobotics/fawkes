
/***************************************************************************
 *  msl2010.h - Fawkes mid-size refbox 2010 protocol repeater
 *
 *  Created: Wed Apr 01 10:36:08 2009
 *  Copyright  2009  Stefan Schiffer [stefanschiffer.de]
 *             2010  Tim Niemueller [www.niemueller.de]
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

#ifndef _PLUGINS_REFBOXCOMM_PROCESSOR_MSL2010_H_
#define _PLUGINS_REFBOXCOMM_PROCESSOR_MSL2010_H_

#include "processor.h"
#include "state_handler.h"

#include <cstddef>

namespace fawkes {
  class MulticastDatagramSocket;
  class Logger;
}

namespace xmlpp {
  class DomParser;
  class Node;
}

class Msl2010RefBoxProcessor : public RefBoxProcessor
{
 public:
  Msl2010RefBoxProcessor(fawkes::Logger *logger,
			 const char *refbox_host, unsigned short int refbox_port);
  ~Msl2010RefBoxProcessor();

  bool check_connection();
  void refbox_process();

 private:
  void process_string(char *buf, size_t len);
  void reconnect();

 private:
  fawkes::Logger *logger_;
  fawkes::MulticastDatagramSocket *s_;

  unsigned int score_cyan_;
  unsigned int score_magenta_;

  const char *name_;

  bool quit_;
  bool connection_died_;

  char *refbox_host_;
  unsigned short int refbox_port_;


  xmlpp::DomParser *dom;
  xmlpp::Node      *root;

};

#endif
