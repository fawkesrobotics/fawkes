
/***************************************************************************
 *  field_view.cpp - Draws the field and the robots on it
 *
 *  Created: Wed April 09 21:00:49 2008
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

#include <tools/worldinfo_viewer/field_view.h>
#include <cairomm/context.h>

#include <cstdio>

using namespace std;

/** @class FieldView <tools/worldinfo_viewer/field_view.h>
 * Drawing widget that draws an (MSL-) soccer field with robots, opponentes, and balls.
 * @author Daniel Beck
 */

/** Constructor. */
FieldView::FieldView()
{
  m_robot_id = 0;
}

/** Destructor. */
FieldView::~FieldView()
{
}

/** Reset all posititions. */
void
FieldView::reset()
{
  m_robot_positions.clear();
  m_ball_positions.clear();
  //m_opponent_positions.clear();
}

/** Set the position of a robot.
 * @param name the hostname of the robot
 * @param glob_x the global x-coordinate of the robot's position
 * @param glob_y the gloabl y-coordinate of the robot's position
 * @param theta the global orienation of the robot
 */
void
FieldView::set_robot_pos(const char* name, float glob_x, float glob_y, float theta)
{
  unsigned int id;
  string robot_name(name);
  if ( m_robots.find(robot_name) != m_robots.end() )
    {
      id = m_robots[robot_name];
    }
  else
    {
      id = m_robot_id++;
      m_robots[robot_name] = id;
    }
  
  // TODO: theta
  HomVector pos(glob_x, glob_y);
  m_robot_positions[id] = pos;
}

/** Set the position of the ball as it is estimated by the given robot.
 * @param name the hostname of the robot
 * @param rel_x the relative x-coordinate of the estimation of the ball position
 * @param rel_y the relative y-coordinate of the estimation of the ball position
 */
void
FieldView::set_ball_pos(const char* name, float rel_x, float rel_y)
{
  unsigned int id;
  string robot_name(name);
  if ( m_robots.find(robot_name) != m_robots.end() )
    {
      id = m_robots[robot_name];
    }
  else
    {
      id = m_robot_id++;
      m_robots[robot_name] = id;
    }

  HomVector pos(rel_x, rel_y);
  m_ball_positions[id] = pos;
}

// void
// FieldView::set_opponent_pos(const char* robot_name, float rel_x, float rel_y)
// {
//   unsigned int id;
//   string robot_name(name);
//   if ( m_robots.find(robot_name) != m_robots.end() )
//     {
//       id = m_robots[robot_name];
//     }
//   else
//     {
//       id = m_robot_id++;
//       m_robots[robot_name] = id;
//     }

//   HomVector pos(rel_x, rel_y);
//   m_opponent_positions[id] = pos;
// }

/** Overloaded signal handler.
 * This is were the drawing action happens.
 * @param event the event that triggered the signal
 * @return always true
 */
bool
FieldView::on_expose_event(GdkEventExpose* event)
{
  Glib::RefPtr<Gdk::Window> window = get_window();
  if (window)
    {
      Gtk::Allocation allocation = get_allocation();
      const int width = allocation.get_width();
      const int height = allocation.get_height();
      
      Cairo::RefPtr<Cairo::Context> context = window->create_cairo_context();
      
      if (event)
	{
	  context->rectangle(event->area.x, event->area.y,
			event->area.width, event->area.height);
	  context->clip();
	}
      
      // setup
      float unit;
      if ( (width / 22.0) <= (height / 16.0) )
	{ unit = width / 22.0; }
      else
	{ unit = height / 16.0; }

      context->translate( width / 2.0, height / 2.0 );
      context->scale(unit, -unit);

      draw_field_msl(context);

      std::map<std::string, unsigned int>::iterator rit;
      for (rit = m_robots.begin(); rit != m_robots.end(); ++rit)
	{
	  HomVector pos;
	  HomVector robot_pos;

	  // robot
	  robot_pos = m_robot_positions[rit->second];
	  draw_robot(context, robot_pos.x(), robot_pos.y(), 0.0 /* TODO */);

	  // ball
	  pos = m_ball_positions[rit->second];
	  draw_ball(context, pos.x(), pos.y());

	  // opponents
	  // TODO
	}
    }
  
  return true;
}

void
FieldView::draw_field_msl(Cairo::RefPtr<Cairo::Context> context)
{
  context->save();

  // background
  context->save();
  context->set_source_rgb(48.0 / 255.0, 215.0 / 255.0, 31.0 / 255.0);
  context->paint();
  context->restore();
      
  context->save();
  context->set_line_width(0.125);
  context->set_source_rgb(1.0, 1.0, 1.0);
  
  // center circle
  context->arc(0.0, 0.0, 2.0, 0.0, 2 * M_PI);
  
  // corner arcs
  context->move_to(9.0, 6.0);
  context->arc(9.0, 6.0, 0.75, M_PI, -M_PI/2.0);
  context->move_to(9.0,-6.0);
  context->arc(9.0, -6.0, 0.75, M_PI/2.0, M_PI);
  context->move_to(-9.0, -6.0);
  context->arc(-9.0, -6.0, 0.75, 0.0, M_PI/2.0);
  context->move_to(-9.0, 6.0);
  context->arc(-9.0, 6.0, 0.75, -M_PI/2.0, 0.0);
  
  // goals
  context->move_to(9.0, 1.0625);
  context->line_to(9.5, 1.0625);
  context->line_to(9.5,-1.0625);
  context->line_to(9.0,-1.0625);
  context->move_to(-9.0, 1.0625);
  context->line_to(-9.5, 1.0625);
  context->line_to(-9.5,-1.0625);
  context->line_to(-9.0,-1.0625);
  
  // lines
  context->save();
  context->set_line_cap(Cairo::LINE_CAP_SQUARE);
  context->move_to( 0.0, 6.0);
  context->line_to( 0.0,-6.0);
  context->move_to( 9.0, 6.0);
  context->line_to( 9.0,-6.0);
  context->line_to(-9.0,-6.0);
  context->line_to(-9.0, 6.0);
  context->close_path();
  context->restore();
  
  // goal areas
  context->move_to(9.0, 1.75);
  context->line_to(8.25, 1.75);
  context->line_to(8.25,-1.75);
  context->line_to(9.0,-1.75);
  context->move_to(-9.0, 1.75);
  context->line_to(-8.25, 1.75);
  context->line_to(-8.25,-1.75);
  context->line_to(-9.0,-1.75);
  
  // penalty areas
  context->move_to(9.0, 3.25);
  context->line_to(6.75, 3.25);
  context->line_to(6.75,-3.25);
  context->line_to(9.0,-3.25);
  context->move_to(-9.0, 3.25);
  context->line_to(-6.75, 3.25);
  context->line_to(-6.75,-3.25);
  context->line_to(-9.0,-3.25);
  
  // marks
  context->move_to(0.0, 0.0);
  context->arc(0.0, 0.0, 0.05, 0.0, 2.0 * M_PI);
  context->move_to(6.0, 0.0);
  context->arc(6.0, 0.0, 0.05, 0.0, 2.0 * M_PI);
  context->move_to(-6.0, 0.0);
  context->arc(-6.0, 0.0, 0.05, 0.0, 2.0 * M_PI);
  context->stroke();
    
  context->restore();
}

void
FieldView::draw_robot(Cairo::RefPtr<Cairo::Context> context, float x, float y, float ori)
{
  context->save();
  context->set_source_rgb(0.7, 0.7, 0.7);
  context->set_line_width(0.05);
  context->move_to(x, y);
  context->arc(x, y, 0.3, ori, 2*M_PI);
  context->stroke();
  context->restore();
}

void
FieldView::draw_obstacle(Cairo::RefPtr<Cairo::Context> context, float x, float y, float extend)
{
  context->save();
  context->set_line_width(0.02);
  context->save();
  context->arc(x, y, extend, 0.0, 2.0 * M_PI);
  context->set_source_rgba(0.0, 0.0, 0.0, 0.6);
  context->fill_preserve();
  context->restore();
  context->stroke();
  context->restore();
}

void
FieldView::draw_ball(Cairo::RefPtr<Cairo::Context> context, float x, float y)
{
  context->save();
  context->set_source_rgb(1.0, 0.3, 0.0);
  context->arc(x, y, 0.15, 0.0, 2.0 * M_PI);
  context->fill();
  context->restore();
}
