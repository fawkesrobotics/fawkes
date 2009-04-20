
/***************************************************************************
 *  field_view.h - Draws the field and the robots on it
 *
 *  Created: Wed April 09 20:58:20 2008
 *  Copyright  2008  Daniel Beck
 *
 *  $Id$
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

#ifndef __TOOL_WORLDINFO_VIEWER_FIELD_VIEW_H_
#define __TOOL_WORLDINFO_VIEWER_FIELD_VIEW_H_

#include <gtkmm/drawingarea.h>
#include <geometry/hom_vector.h>
#include <geometry/hom_point.h>
#include <geometry/hom_pose.h>
#include <map>

namespace fawkes {
  class WorldInfoDataContainer;
}

class FieldView : public Gtk::DrawingArea
{
 public:
  FieldView( fawkes::WorldInfoDataContainer* data,
	     bool show_pose = true,
	     bool show_ball = true,
	     bool show_opponents = false );
  virtual ~FieldView();

  bool toggle_show_pose( Glib::ustring name );
  bool toggle_show_ball( Glib::ustring name );
  bool toggle_show_opponents( Glib::ustring name );

  void remove_host( Glib::ustring name );

 protected:
  virtual bool on_expose_event(GdkEventExpose* event);

 private:
  void draw_field_msl( Cairo::RefPtr<Cairo::Context> context );
  void draw_robot( Cairo::RefPtr<Cairo::Context> context, 
		   float x, float y, float ori,
		   Glib::ustring name );
  void draw_obstacle( Cairo::RefPtr<Cairo::Context> context, 
		      float x, float y, float extend );
  void draw_ball( Cairo::RefPtr<Cairo::Context> context, 
		  float ball_x, float ball_y, float bot_x, float bot_y );

  fawkes::WorldInfoDataContainer* m_data_container;

  std::map< Glib::ustring, bool > m_show_pose;
  std::map< Glib::ustring, bool > m_show_ball;
  std::map< Glib::ustring, bool > m_show_opponents;
  bool m_show_pose_default;
  bool m_show_ball_default;
  bool m_show_opponents_default;
};

#endif /* __TOOL_WORLDINFO_VIEWER_FIELD_VIEW_H_ */
