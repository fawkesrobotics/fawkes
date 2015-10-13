
/***************************************************************************
 *  eclipse_debugger.h - Eclipse Debugger Tool
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

#ifndef __TOOLS_ECLIPSEDEBUGGER_ECLIPSEDEBUGGER_H_
#define __TOOLS_ECLIPSEDEBUGGER_ECLIPSEDEBUGGER_H_

#include <interfaces/EclipseDebuggerInterface.h>

#include <gui_utils/connection_dispatcher.h>
#include <gtkmm.h>



namespace fawkes {
  class BlackBoard;
  class InterfaceDispatcher;
  class LogView;
}



class EclipseDebugger : public Gtk::Window
{
  public: 
    EclipseDebugger(BaseObjectType* cobject,
                    const Glib::RefPtr<Gtk::Builder> &builder);
    ~EclipseDebugger();
  
  private:
    void establish_connection();
    void on_connect();
    void on_disconnect();
    void close_bb();
    fawkes::BlackBoard *bb;
    
    fawkes::EclipseDebuggerInterface *__debugger_if;
    fawkes::ConnectionDispatcher connection_dispatcher;

};

#endif
