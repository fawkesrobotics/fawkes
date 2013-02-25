
/***************************************************************************
 *  eclipse_thread.h - Fawkes ECLiPSe Thread
 *
 *  Created: Wed Jul 16 10:20:51 2009
 *  Copyright  2009  Daniel Beck
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

#ifndef __PLUGINS_ECLIPSE_CLP_ECLIPSE_THREAD_H_
#define __PLUGINS_ECLIPSE_CLP_ECLIPSE_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>



#include <map>
#include <string>
#include <vector>

namespace fawkes {
  class Interface;
}

class EclipseAgentThread 
: public fawkes::Thread,
  public fawkes::BlackBoardAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::LoggingAspect
{
 public:
  EclipseAgentThread();
  virtual ~EclipseAgentThread();

  virtual void init();
  virtual void finalize();
  virtual void once();

  void post_event( const char* );
  void read_interfaces();
  void write_interfaces();

  fawkes::Interface* get_registered_interface( const char* id );
  fawkes::Logger*    get_logger();

  static EclipseAgentThread* instance();

 private: /* methods */
  bool load_file( const char* filename );

 private: /* members */
  static EclipseAgentThread* m_instance;

  bool m_initialized;
  std::string agent;

  std::vector< fawkes::Interface* > m_reading_ifaces;
  std::vector< fawkes::Interface* > m_writing_ifaces;

  std::multimap< std::string, fawkes::Interface* > m_registered_interfaces;
};

#endif /* __PLUGINS_ECLIPSE_CLP_ECLIPSE_THREAD_H_ */
