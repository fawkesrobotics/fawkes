/***************************************************************************
 *  eclipse_debugger.cpp -  Eclipse Debugger Tool
 *
 *  Created: Mon Feb 25 14:22:00 2013
 *  Copyright  2013  Gesche Gierse
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


#include "eclipse_debugger.h"

#include <utils/system/argparser.h>
#include <blackboard/remote.h>
#include <netcomm/fawkes/client.h>

#include <gui_utils/logview.h>
#include <gui_utils/service_chooser_dialog.h>
#include <gui_utils/interface_dispatcher.h>

#include <cstring>
#include <string>
#include <sstream>

using namespace fawkes;

/** @class EclipseDebugger "eclipse_debugger.h"
 * ECLiPSe-clp Debugger GUI Wrapper.
 * Can connect remotely with (embedded) ECLiPSe-clp session and
 * starts the tkeclipse-clp graphical debugger. Has to be supported
 * from the ECLiPSe program.
 * @author Gesche Gierse
 */


/** Constructor.
 * @param cobject C base object
 * @param builder Gtk Builder
 */
EclipseDebugger::EclipseDebugger(BaseObjectType* cobject,
                    const Glib::RefPtr<Gtk::Builder> &builder)
{
  bb = NULL;
  
  connection_dispatcher.signal_connected().connect(sigc::mem_fun(*this, &EclipseDebugger::on_connect));
  connection_dispatcher.signal_disconnected().connect(sigc::mem_fun(*this, &EclipseDebugger::on_disconnect));

  establish_connection();
}

/** Destructor. */
EclipseDebugger::~EclipseDebugger()
{

}

void
EclipseDebugger::establish_connection()
{
  if ( ! connection_dispatcher.get_client()->connected() ) {
    ServiceChooserDialog ssd(*this, connection_dispatcher.get_client());
    ssd.run_and_connect();
  } else {
    connection_dispatcher.get_client()->disconnect();
  } 
}

/** Event handler for connected event. */
void
EclipseDebugger::on_connect()
{
  try {
    if ( ! bb ) {
      bb           = new RemoteBlackBoard(connection_dispatcher.get_client());
      __debugger_if = bb->open_for_reading<EclipseDebuggerInterface>("readylog_connect");
      EclipseDebuggerInterface::ConnectionMessage *cm = new EclipseDebuggerInterface::ConnectionMessage();
      __debugger_if->msgq_enqueue(cm);
      sleep(1);
      __debugger_if->read();
      char* host = __debugger_if->host();
      unsigned int port = __debugger_if->port();
      std::stringstream portstr;
      portstr << port;
      execlp("tktools-clp", "tktools-clp", "-h", host, "-p", portstr.str().c_str(), (char *) 0);
    }

  } catch (Exception &e) {
    Glib::ustring message = *(e.begin());
    Gtk::MessageDialog md(*this, message, /* markup */ false,
			  Gtk::MESSAGE_ERROR, Gtk::BUTTONS_OK,
			  /* modal */ true);
    md.set_title("BlackBoard connection failed");
    md.run();

    close_bb();
    connection_dispatcher.get_client()->disconnect();
  }
}


/** Event handler for disconnected event. */
void
EclipseDebugger::on_disconnect()
{
  close_bb();
}


void
EclipseDebugger::close_bb()
{
  if ( bb ) {
    delete bb;
    bb = NULL;
  }
}
