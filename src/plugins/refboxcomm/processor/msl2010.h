
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

#ifndef __PLUGINS_REFBOXCOMM_PROCESSOR_MSL2010_H_
#define __PLUGINS_REFBOXCOMM_PROCESSOR_MSL2010_H_

#include "processor.h"
#include "state_handler.h"

#include <cstdlib>
#include <string>
#include <vector>

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
  fawkes::Logger *__logger;
  fawkes::MulticastDatagramSocket *__s;

  unsigned int __score_cyan;
  unsigned int __score_magenta;

  const char *__name;

  bool __quit;
  bool __connection_died;

  char *__refbox_host;
  unsigned short int __refbox_port;


  xmlpp::DomParser *dom;
  xmlpp::Node      *root;

};

#endif
