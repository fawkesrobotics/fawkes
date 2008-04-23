
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

class FieldView : public Gtk::DrawingArea
{
 public:
  FieldView();
  virtual ~FieldView();

  void reset();

  void set_robot_pose(const char* name, float glob_x, float glob_y, float theta);
  void set_ball_pos(const char* robot_name, float rel_x, float rel_y);
/*   void set_opponent_pos(const char* robot_name, float rel_x, float rel_y); */

 protected:
  virtual bool on_expose_event(GdkEventExpose* event);

 private:
  void draw_field_msl(Cairo::RefPtr<Cairo::Context> context);
  void draw_robot(Cairo::RefPtr<Cairo::Context> context, 
		  float x, float y, float ori);
  void draw_obstacle(Cairo::RefPtr<Cairo::Context> context, 
		     float x, float y, float extend);
  void draw_ball(Cairo::RefPtr<Cairo::Context> context, 
		 float ball_x, float ball_y, float bot_x, float bot_y);

  std::map<std::string, unsigned int> m_robots;
  std::map<unsigned int, HomPose> m_robot_poses;
/*   std::map<unsigned int, std::vector<HomVector> > m_opponent_positions; */
  std::map<unsigned int, HomVector> m_ball_positions;

  unsigned int m_robot_id;
};

#endif /* __TOOL_WORLDINFO_VIEWER_FIELD_VIEW_H_ */
