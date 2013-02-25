

#include "eclipse_debugger.h"

#include <utils/system/argparser.h>
#include <blackboard/remote.h>
#include <netcomm/fawkes/client.h>

#include <gui_utils/logview.h>
#include <gui_utils/throbber.h>
#include <gui_utils/service_chooser_dialog.h>
#include <gui_utils/interface_dispatcher.h>

#include <cstring>
#include <string>
#include <sstream>

using namespace fawkes;

EclipseDebugger::EclipseDebugger(BaseObjectType* cobject,
                    const Glib::RefPtr<Gtk::Builder> &builder)
{
  bb = NULL;
  
  connection_dispatcher.signal_connected().connect(sigc::mem_fun(*this, &EclipseDebugger::on_connect));
  connection_dispatcher.signal_disconnected().connect(sigc::mem_fun(*this, &EclipseDebugger::on_disconnect));

  establish_connection();
}


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
      std::stringstream command;
      command << "tktools -h " << host << " -p " << port;      
      system(command.str().c_str());
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
