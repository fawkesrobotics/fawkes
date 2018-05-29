
/***************************************************************************
 *  rrdweb_processor.h - RRD web request processor
 *
 *  Created: Tue Dec 21 01:11:01 2010
 *  Copyright  2006-2010  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_RRDWEB_RRDWEB_PROCESSOR_H_
#define __PLUGINS_RRDWEB_RRDWEB_PROCESSOR_H_

namespace fawkes {
  class RRDManager;
  class Logger;
  class WebReply;
  class WebRequest;
}

class RRDWebRequestProcessor
{
 public:
	RRDWebRequestProcessor(fawkes::RRDManager *rrd_manager, fawkes::Logger *logger);

  virtual ~RRDWebRequestProcessor();

  fawkes::WebReply * process_overview();
  fawkes::WebReply * process_graph(const fawkes::WebRequest *request);

 private:
  fawkes::RRDManager   *__rrd_man;
  fawkes::Logger       *__logger;
};

#endif
