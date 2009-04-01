
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

#include "field_view.h"

#include <worldinfo_utils/data_container.h>
#include <cairomm/context.h>

#include <map>
#include <cstdio>

using namespace std;
using namespace fawkes;

/** @class FieldView <tools/worldinfo_viewer/field_view.h>
 * Drawing widget that draws an (MSL-) soccer field with robots,
 * opponents, and balls.
 * @author Daniel Beck
 */

/** Constructor.
 * @param show_pose default value for show pose
 * @param show_ball default value for show ball
 * @param show_opponents default value for show opponents 
 * @param data pointer to a WorldInfoDataContainer that is used as the
 * data source
 */
FieldView::FieldView( WorldInfoDataContainer* data,
		      bool show_pose,
		      bool show_ball,
		      bool show_opponents )
{
  m_show_pose_default      = show_pose;
  m_show_ball_default      = show_ball;
  m_show_opponents_default = show_opponents;

  m_data_container = data;
}

/** Destructor. */
FieldView::~FieldView()
{
}

/** Toggle whether to show the pose of the specified robot.
 * @param name the hostname of the robot
 * @return true if the pose wasn't shown before, false otherwise
 */
bool
FieldView::toggle_show_pose( Glib::ustring name )
{
  std::map< Glib::ustring, bool >::iterator iter = m_show_pose.find( name );
  if ( iter != m_show_pose.end() )
  { 
    iter->second = iter->second ? false : true; 
    return iter->second;
  }
  else
  {
    m_show_pose[ name ] = m_show_pose_default;
    return m_show_pose_default;
  }
}

/** Toggle whether to show the ball detected by the specified robot.
 * @param name the hostname of the robot
 * @return true if the ball wasn't shown before, false otherwise
 */
bool
FieldView::toggle_show_ball( Glib::ustring name )
{
  std::map< Glib::ustring, bool >::iterator iter = m_show_ball.find( name );
  if ( iter != m_show_ball.end() )
  {
    iter->second = iter->second ? false : true;
    return iter->second;
  }
  else
  {
    m_show_ball[ name ] = m_show_ball_default;
    return m_show_ball_default;
  }
}

/** Toggle whether to show the opponents seen by the specified robot.
 * @param name the hostname of the robot
 * @return true if the opponents weren't shown before, false otherwise
 */
bool
FieldView::toggle_show_opponents( Glib::ustring name )
{
  std::map< Glib::ustring, bool >::iterator iter = m_show_opponents.find( name );
  if ( iter != m_show_opponents.end() )
  {
    iter->second = iter->second ? false : true;
    return iter->second;
  }
  else
  {
    m_show_opponents[ name ] = m_show_opponents_default;
    return m_show_opponents_default;
  }
}

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
    const int width  = allocation.get_width();
    const int height = allocation.get_height();
    
    Cairo::RefPtr<Cairo::Context> context = window->create_cairo_context();
    
    if (event)
    {
      context->rectangle( event->area.x, event->area.y,
			  event->area.width, event->area.height );
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
    
    list<string> hosts = m_data_container->get_hosts();
    
    for ( list<string>::iterator i = hosts.begin();
	  i != hosts.end();
	  ++i )
    {
      const char* host = i->c_str();
      
      HomPose pose;
      HomPoint ball_pos;
      std::map<unsigned int, HomPoint> opp_positions;
      
      bool show_pose;
      bool show_ball;
      bool show_opponents;
      std::map< Glib::ustring, bool >::iterator iter;
      
      iter = m_show_pose.find( *i );
      if ( iter == m_show_pose.end() )
      { show_pose = m_show_pose_default; }
      else
      { show_pose = iter->second; }
      
      iter = m_show_ball.find( *i );
      if ( iter == m_show_ball.end() )
      { show_ball = m_show_ball_default; }
      else
      { show_ball = iter->second; }
      
      iter = m_show_opponents.find( *i );
      if ( iter == m_show_opponents.end() )
      { show_opponents = m_show_opponents_default; }
      else
      { show_opponents = iter->second; }
      
      
      if ( m_data_container->get_robot_pose( host, pose ) )
      {
	if ( show_pose )
	{ draw_robot( context, pose.x(), pose.y(), pose.yaw(), host ); }
	
	if ( m_data_container->get_ball_pos_global( host, ball_pos ) &&
	     show_ball )
	{ 
	  draw_ball( context, ball_pos.x(), ball_pos.y(),
		     pose.x(), pose.y() ); 
	}
	
	// TODO
// 	if ( m_data_container->get_opponenet_pos( host, opp_positions ) &&
// 	     show_opponents )
// 	{ 
// 	}
      }
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
FieldView::draw_robot( Cairo::RefPtr<Cairo::Context> context,
		       float x,
		       float y,
		       float ori,
		       Glib::ustring name )
{
  context->save();
  context->set_source_rgb(0.2, 0.2, 0.2);
  context->set_line_width(0.05);
  context->move_to(x, y);
  context->arc(x, y, 0.3, ori, 2*M_PI + ori);
  context->stroke();
  context->restore();

  context->save();
  context->select_font_face( "Sans",
			     Cairo::FONT_SLANT_NORMAL,
			     Cairo::FONT_WEIGHT_NORMAL );
  context->set_font_size( 4 );
  context->scale(0.1, -0.1);
  
  Cairo::TextExtents extents;
  context->get_text_extents( name.c_str(), extents );

  context->move_to(  10 * x - extents.width/2.0  - extents.x_bearing,
		    -10 * y - extents.height/2.0 - extents.y_bearing + 8 );
  context->show_text( name.c_str() );

  char* pos;
  asprintf( &pos, "%.2f, %.2f [%.2f]", x, y, ori );
  context->get_text_extents( pos, extents );

  context->move_to(  10 * x -  extents.width/2.0  - extents.x_bearing,
		    -10 * y - extents.height/2.0 - extents.y_bearing + 12 );
  context->show_text( pos );

  context->stroke();
  context->restore();

  free( pos );
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
FieldView::draw_ball( Cairo::RefPtr<Cairo::Context> context, 
		      float ball_x, float ball_y,
		      float bot_x, float bot_y )
{
  context->save();
  context->set_source_rgb(1.0, 0.3, 0.0);
  context->set_line_width(0.05);
  context->move_to(bot_x, bot_y);
  context->line_to(ball_x, ball_y);
  context->stroke();
  context->arc(ball_x, ball_y, 0.15, 0.0, 2.0 * M_PI);
  context->fill();
  context->restore();
}
