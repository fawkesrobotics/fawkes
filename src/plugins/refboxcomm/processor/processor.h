
/***************************************************************************
 *  processor.h - Fawkes RefBox Processor Pure Virtual Class
 *
 *  Created: Sun Apr 19 14:15:08 2009 (on way to German Open 2009)
 *  Copyright  2009  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_REFBOXCOMM_PROCESSOR_PROCESSOR_H_
#define __PLUGINS_REFBOXCOMM_PROCESSOR_PROCESSOR_H_

class RefBoxStateHandler;

class RefBoxProcessor
{
 public:
  virtual ~RefBoxProcessor();

  virtual bool check_connection() = 0;
  virtual void refbox_process() = 0;

  virtual void set_handler(RefBoxStateHandler *rsh);

 protected:
  /** Refbox state handler, set via set_handler() */
  RefBoxStateHandler  *_rsh;
};

#endif
