/***************************************************************************
 *  navigator_gui.h - Navigator GUI
 *
 *  Generated: Fri Jun 01 11:41:34 2007
 *  Copyright  2007  Martin Liebenberg
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */
 
#ifndef __TOOLS_NAVIGATOR_GUI_NAVIGATOR_GUI_H_
#define __TOOLS_NAVIGATOR_GUI_NAVIGATOR_GUI_H_

#include <plugins/navigator/libnavi/npoint.h>

#include <libs/netcomm/fawkes/client_handler.h>
#include <libs/core/utils/lock_list.h>

#include <gtkmm/drawingarea.h>

#include <vector>

class FawkesNetworkClient;
class FawkesNetworkMessage;
class NLine;
class Mutex;

namespace Gtk
{
  class Button;
  class HScale;
  class Adjustment;
}



class NavigatorGUI : public  Gtk::DrawingArea, public FawkesNetworkClientHandler
{
 public:

  NavigatorGUI(const char *host_name);
  virtual ~NavigatorGUI();


  Gtk::Button *control_button;
  Gtk::HScale *velocity_HScale;
 private:
   
  Gtk::Adjustment *velocity_adjustment;

  double window_width;
  double window_height;
  double scanning_area_width;
  double scanning_area_height;
        
  double point_radius;
  double robot_radius;
        
  NPoint target_point;
  double robot_orientation;
  bool rotate;
  int old_x_coordinate;
        
  LockList<NPoint*> points;
  LockList<NLine*> lines;
        
  Mutex *target_mutex;
        
  double velocity;
        
  unsigned int value;
        
  FawkesNetworkClient *net_client;
  void deregistered();
  void inboundReceived(FawkesNetworkMessage *m);
        
  bool on_idle();
  bool on_expose_event(GdkEventExpose* event);
  bool on_button_press_event(GdkEventButton* event);
  bool on_button_release_event(GdkEventButton* event);
  bool on_motion_notify_event(GdkEventMotion* event);
  void on_control_button_clicked();
  void on_velocity_value_changed();
  bool control;
};

#endif /*NAVIGATOR_GUI_H_*/
