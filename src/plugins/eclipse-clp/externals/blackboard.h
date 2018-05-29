
/***************************************************************************
 *  blackboard.h - External predicates to remotely access the Fawkes
 *                 blackboard
 *
 *  Created: Wed Mar 09 16:57:03 2011
 *  Copyright  2011  Daniel Beck
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

#ifndef __ECLIPSE_EXTERNALS_BLACKBOARD_H_
#define __ECLIPSE_EXTERNALS_BLACKBOARD_H_

#include <blackboard/remote.h>
#include <logging/logger.h>
#include <vector>

#include <cstdio>

/** @class fawkes::EclExternalBlackBoard
 * Wrapper class for using the blackboard in the implementation of the external
 * predicates.
 * @author Daniel Beck
 */
namespace fawkes
{
class EclExternalBlackBoard
{
private:
  /** Constructor.
   * @param blackboard blackboard to use to open interfaces
   */
  EclExternalBlackBoard(BlackBoard *blackboard, Logger *logger);
public:
  /** Destructor. */
  ~EclExternalBlackBoard();

  static void create_initial_object(BlackBoard *bb, Logger *logger);
  static void cleanup_instance();
  static EclExternalBlackBoard* instance();

  static BlackBoard* blackboard_instance();
  std::map<std::string, Interface *> & interfaces();

  /**
   * @return A pointer to the plugin-central logger
   */
  static Logger *logger() { return m_logger; }

  /**
   * @return Name for logging
   */
  static const char *name() { return "EclExternalBlackBoard"; }

private:
  static EclExternalBlackBoard *      m_instance;
  std::map<std::string, Interface *>  m_interfaces;
  static BlackBoard *                 m_blackboard;
  static Logger *                     m_logger;
};
}


extern "C" int p_bb_open_interface();
extern "C" int p_bb_close_interface();

extern "C" int p_bb_has_writer();
extern "C" int p_bb_instance_serial();

extern "C" int p_bb_read_interfaces();
extern "C" int p_bb_read_interface();
extern "C" int p_bb_write_interfaces();
extern "C" int p_bb_write_interface();
extern "C" int p_bb_interface_changed();

extern "C" int p_bb_get();
extern "C" int p_bb_set();

extern "C" int p_bb_send_message();
extern "C" int p_bb_recv_messages();

extern "C" int p_bb_observe_pattern();
extern "C" int p_bb_listen_for_change();

#endif

