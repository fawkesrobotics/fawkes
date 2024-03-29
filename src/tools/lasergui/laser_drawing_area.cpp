
/***************************************************************************
 *  laser_drawing_area.cpp - Laser drawing area derived from Gtk::DrawingArea
 *
 *  Created: Thu Oct 09 18:20:21 2008
 *  Copyright  2008-2010  Tim Niemueller [www.niemueller.de]
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

#include "laser_drawing_area.h"

#include "visdisplay.h"

#include <gui_utils/robot/drawer.h>
#include <interfaces/Laser1080Interface.h>
#include <interfaces/Laser360Interface.h>
#include <interfaces/Laser720Interface.h>
#include <interfaces/ObjectPositionInterface.h>
#include <interfaces/VisualDisplay2DInterface.h>
#include <utils/math/angle.h>
#include <utils/misc/string_conversions.h>

#include <algorithm>
#include <cmath>
#include <cstdio>

//#define LASERGUI_DEBUG_PRINT_TRACKS
#define CFG_PRINT_NR_TRACKELEMENTS 5

using namespace fawkes;

/** @class LaserDrawingArea "laser_drawing_area.h"
 * Laser drawing area.
 * Derived version of Gtk::DrawingArea that renders values of a laser interface.
 * @author Tim Niemueller
 */

/** Constructor.
 * Special ctor to be used with Gtk::Builder's get_widget_derived().
 * @param cobject Gtk C object
 * @param builder Gtk Builder
 */
LaserDrawingArea::LaserDrawingArea(BaseObjectType                   *cobject,
                                   const Glib::RefPtr<Gtk::Builder> &builder)
: Gtk::DrawingArea(cobject)
{
	draw_mode_             = MODE_LINES;
	zoom_factor_           = 50;
	l_objpos_if_persons_   = NULL;
	l_objpos_if_legs_      = NULL;
	l_objpos_if_misc_      = NULL;
	laser_segmentation_if_ = NULL;
	l_track_if_            = NULL;
	target_if_             = NULL;
	switch_if_             = NULL;
	line_if_               = NULL;
	visdisp_if_            = NULL;
	robot_drawer_          = NULL;
	resolution_            = 1;
	rotation_              = 0;
	break_drawing_         = false;
	first_draw_            = true;
	connected_             = false;

	visdisp_ = new VisualDisplay2D();

	add_events(Gdk::SCROLL_MASK | Gdk::BUTTON_MOTION_MASK | Gdk::BUTTON_PRESS_MASK);

#if GTK_VERSION_LT(3, 0)
	signal_expose_event().connect(sigc::mem_fun(*this, &LaserDrawingArea::on_expose_event));
	signal_button_press_event().connect(
	  sigc::mem_fun(*this, &LaserDrawingArea::on_button_press_event));
	signal_motion_notify_event().connect(
	  sigc::mem_fun(*this, &LaserDrawingArea::on_motion_notify_event));
#endif
	//Glib::RefPtr<Gdk::Window> window = get_window();
}

/** Constructor. */
LaserDrawingArea::LaserDrawingArea()
{
	draw_mode_             = MODE_LINES;
	zoom_factor_           = 50;
	l_objpos_if_persons_   = NULL;
	l_objpos_if_legs_      = NULL;
	l_objpos_if_misc_      = NULL;
	laser_segmentation_if_ = NULL;
	l_track_if_            = NULL;
	target_if_             = NULL;
	switch_if_             = NULL;
	line_if_               = NULL;
	visdisp_if_            = NULL;
	robot_drawer_          = NULL;
	resolution_            = 1;
	rotation_              = 0;
	break_drawing_         = false;

	visdisp_ = new VisualDisplay2D();

	add_events(Gdk::SCROLL_MASK | Gdk::BUTTON_MOTION_MASK);

#if GTK_VERSION_LT(3, 0)
	signal_expose_event().connect(sigc::mem_fun(*this, &LaserDrawingArea::on_expose_event));
	signal_button_press_event().connect(
	  sigc::mem_fun(*this, &LaserDrawingArea::on_button_press_event));
	signal_motion_notify_event().connect(
	  sigc::mem_fun(*this, &LaserDrawingArea::on_motion_notify_event));
#endif
}

/** Destructor. */
LaserDrawingArea::~LaserDrawingArea()
{
	delete visdisp_;
}

/** Set ObjectPosition interfaces.
 * @param  l_objpos_if_persons list of objectposition interfaces for persons
 * @param  l_objpos_if_legs list of objectposition interfaces for legs
 * @param l_objpos_if_misc list of objectposition interfaces for miscellanous objects
 * @param laser_segmentation_if Laser interface indicating the segmentation-borfers of the legtracker
 * @param l_track_if list of track interfaces
 * @param target_if the current target
 * @param switch_if used to indicate that a drawing-run is finish (so e.g. new data can be sent)
 */
void
LaserDrawingArea::set_objpos_if(std::list<fawkes::ObjectPositionInterface *> *l_objpos_if_persons,
                                std::list<fawkes::ObjectPositionInterface *> *l_objpos_if_legs,
                                std::list<fawkes::ObjectPositionInterface *> *l_objpos_if_misc,
                                fawkes::Laser720Interface                    *laser_segmentation_if,
                                std::list<fawkes::Position2DTrackInterface *> *l_track_if,
                                fawkes::ObjectPositionInterface               *target_if,
                                fawkes::SwitchInterface                       *switch_if)
{
	l_objpos_if_persons_   = l_objpos_if_persons;
	l_objpos_if_legs_      = l_objpos_if_legs;
	l_objpos_if_misc_      = l_objpos_if_misc;
	laser_segmentation_if_ = laser_segmentation_if;
	l_track_if_            = l_track_if;
	target_if_             = target_if;
	switch_if_             = switch_if;
}

/** Set connection status.
 * @param connected true if connected, false otherwise
 */
void
LaserDrawingArea::set_connected(bool connected)
{
	connected_ = connected;
	queue_draw();
}

/** Set new laser interfaces.
 *
 * This is also the place where colors are determined the following way:
 * <pre>
 *  1   000 ->   0   0   0
 *  2   001 -> 255   0   0
 *  3   010 ->   0 255   0
 *  4   011 -> 255 255   0
 *  5   100 ->   0   0 255
 *  6   101 -> 255   0 255
 *  7   110 -> 255 255   0
 *  8   000 ->   0   0   0
 *  9   001 -> 127   0   0
 * 10   010 ->   0 127   0
 * 11   011 -> 127 127   0
 * 12   100 ->   0   0 127
 * 13   101 -> 127   0 127
 * 14   110 -> 127 127   0
 * ...
 * </pre>
 *
 * @param ifs The interfaces of the lasers that should be visualized.
 */
void
LaserDrawingArea::set_laser_ifs(const std::list<fawkes::Interface *> &ifs)
{
	laser_ifs_.clear();
	unsigned char color_counter = 0;
	unsigned char intensity     = 255;
	for (std::list<fawkes::Interface *>::const_iterator it = ifs.begin(); it != ifs.end(); ++it) {
		if ((color_counter & 0x7) != 0) {
			intensity /= 2;
		}
		Color c;
		c.r                        = ((color_counter & 0x1) != 0) ? intensity : 0;
		c.g                        = ((color_counter & 0x2) != 0) ? intensity : 0;
		c.b                        = ((color_counter & 0x4) != 0) ? intensity : 0;
		const InterfaceColorPair p = std::make_pair(*it, c);
		laser_ifs_.push_back(p);
		++color_counter;
	}
	queue_draw();
}

/** Reset laser interfaces to "no laser available". */
void
LaserDrawingArea::reset_laser_ifs()
{
	laser_ifs_.clear();
	l_objpos_if_persons_   = NULL;
	l_objpos_if_legs_      = NULL;
	l_objpos_if_misc_      = NULL;
	laser_segmentation_if_ = NULL;
	l_track_if_            = NULL;
	target_if_             = NULL;
	switch_if_             = NULL;

	Gtk::Allocation allocation = get_allocation();
	const int       width      = allocation.get_width();
	const int       height     = allocation.get_height();

	xc_          = width / 2;
	yc_          = height / 2;
	zoom_factor_ = 50;
	queue_draw();
}

/** Set line interface.
 * @param line_if interface to use for line data to draw.
 */
void
LaserDrawingArea::set_line_if(ObjectPositionInterface *line_if)
{
	line_if_ = line_if;
}

/** Set visual display interface.
 * @param visdisp_if interface to query for drawing ops
 */
void
LaserDrawingArea::set_visdisp_if(VisualDisplay2DInterface *visdisp_if)
{
	visdisp_if_ = visdisp_if;
	visdisp_->set_interface(visdisp_if_);
}

/** Set robot drawer.
 * @param robot_drawer new robot drawer to use
 */
void
LaserDrawingArea::set_robot_drawer(fawkes::CairoRobotDrawer *robot_drawer)
{
	robot_drawer_ = robot_drawer;
}

/** Set resolution.
 * Every n'th beam will be drawn where n is the resolution.
 * @param resolution new resolution
 */
void
LaserDrawingArea::set_resolution(unsigned int resolution)
{
	resolution_ = resolution;
}

/** Set the drawing mode.
 * @param mode the new drawing mode
 */
void
LaserDrawingArea::set_draw_mode(draw_mode_t mode)
{
	draw_mode_ = mode;
	queue_draw();
}

/** Zoom in.
 * Increases zoom factor by 20, no upper limit.
 */
void
LaserDrawingArea::zoom_in()
{
	zoom_factor_ += 20;
	queue_draw();
}

/** Zoom out.
 * Decreases zoom factor by 20 with a minimum of 1.
 */
void
LaserDrawingArea::zoom_out()
{
	if (zoom_factor_ > 20) {
		zoom_factor_ -= 20;
	} else {
		zoom_factor_ = 1;
	}
	queue_draw();
}

/** Set rotation.
 * @param rot_rad rotation angle in rad
 */
void
LaserDrawingArea::set_rotation(float rot_rad)
{
	rotation_ = rot_rad;
}

bool
LaserDrawingArea::all_laser_ifs_have_writer() const
{
	for (std::list<InterfaceColorPair>::const_iterator it = laser_ifs_.begin();
	     it != laser_ifs_.end();
	     ++it) {
		fawkes::Interface *itf = it->first;
		if (!itf->has_writer()) {
			return false;
		}
	}
	return true;
}

#if GTK_VERSION_GE(3, 0)
/** Expose event handler.
 * @param cr Cairo context for drawing
 * @return signal return value
 */
bool
LaserDrawingArea::on_draw(const Cairo::RefPtr<Cairo::Context> &cr)
#else
/** Expose event handler.
 * @param event event info structure.
 * @return signal return value
 */
bool
LaserDrawingArea::on_expose_event(GdkEventExpose *event)
#endif
{
	// This is where we draw on the window
	Glib::RefPtr<Gdk::Window> window = get_window();
	if (window) {
		Gtk::Allocation allocation = get_allocation();

		if (first_draw_) {
			first_draw_      = false;
			const int width  = allocation.get_width();
			const int height = allocation.get_height();

			// coordinates for the center of the window
			xc_ = width / 2;
			yc_ = height / 2;
		}
#if GTK_VERSION_LT(3, 0)
		Cairo::RefPtr<Cairo::Context> cr = window->create_cairo_context();
#endif
		cr->set_line_width(1.0);

		cr->set_source_rgb(1, 1, 1);
#if GTK_VERSION_LT(3, 0)
		// clip to the area indicated by the expose event so that we only
		// redraw the portion of the window that needs to be redrawn
		cr->rectangle(event->area.x, event->area.y, event->area.width, event->area.height);
		cr->fill_preserve();
		cr->clip();
#else
		cr->paint();
#endif
		cr->set_source_rgb(0, 0, 0);
		//cr->set_source_rgba(0,0,0,1);

		//    last_xc_ += translation_x_;
		//    last_yc_ += translation_y_;
		cr->translate(xc_, yc_);

		cr->save();
		if (!connected_) {
			Cairo::TextExtents te;
			std::string        t = "Not connected to BlackBoard";
			cr->set_source_rgb(1, 0, 0);
			cr->set_font_size(20);
			cr->get_text_extents(t, te);
			cr->move_to(-te.width / 2, -te.height / 2);
			cr->show_text(t);
		} else if (laser_ifs_.empty()) {
			Cairo::TextExtents te;
			std::string        t = "No interface opened";
			cr->set_source_rgb(1, 0, 0);
			cr->set_font_size(20);
			cr->get_text_extents(t, te);
			cr->move_to(-te.width / 2, -te.height / 2);
			cr->show_text(t);
		} else if (!all_laser_ifs_have_writer()) {
			Cairo::TextExtents te;
			std::string        t = "No writer for ";
			for (std::list<InterfaceColorPair>::const_iterator it = laser_ifs_.begin();
			     it != laser_ifs_.end();
			     ++it) {
				fawkes::Interface *itf = it->first;
				if (!itf->has_writer()) {
					t += itf->uid();
					t += ' ';
				}
			}
			cr->set_source_rgb(1, 0, 0);
			cr->set_font_size(20);
			cr->get_text_extents(t, te);
			cr->move_to(-te.width / 2, -te.height / 2);
			cr->show_text(t);
		} else {
			if (!break_drawing_) {
				for (std::list<InterfaceColorPair>::const_iterator it = laser_ifs_.begin();
				     it != laser_ifs_.end();
				     ++it) {
					fawkes::Interface *laser_if = it->first;
					laser_if->read();
				}
			}

			for (std::list<InterfaceColorPair>::const_iterator it = laser_ifs_.begin();
			     it != laser_ifs_.end();
			     ++it) {
				const fawkes::Interface *laser_if = it->first;
				const Color             &color    = it->second;
				cr->save();
				cr->set_source_rgb(color.r, color.g, color.b);
				draw_beams(laser_if, window, cr);
				cr->restore();
			}
			if (robot_drawer_)
				robot_drawer_->draw_robot(window, cr);
			for (std::list<InterfaceColorPair>::const_iterator it = laser_ifs_.begin();
			     it != laser_ifs_.end();
			     ++it) {
				const fawkes::Interface *laser_if = it->first;
				const Color             &color    = it->second;
				cr->save();
				cr->set_source_rgb(color.r, color.g, color.b);
				draw_segments(laser_if, window, cr);
				cr->restore();
			}
			draw_persons_legs(window, cr);

			if (switch_if_ != NULL && switch_if_->has_writer()) {
				SwitchInterface::EnableSwitchMessage *esm = new SwitchInterface::EnableSwitchMessage();
				switch_if_->msgq_enqueue(esm);
			}
		}
		cr->restore();

		cr->save();
		cr->rotate(0.5 * M_PI + rotation_);
		cr->scale(-zoom_factor_, zoom_factor_);
		cr->set_line_width(1. / zoom_factor_);
		if (visdisp_if_) {
			visdisp_->process_messages();
			visdisp_->draw(cr);
		}

		const float radius = 0.01;
		if (line_if_) {
			line_if_->read();
			if (line_if_->has_writer() && line_if_->is_valid() && line_if_->is_visible()) {
				cr->set_source_rgb(1, 0, 0);
				/*
	std::vector<double> dashes(1);
	dashes[0] = 0.1;
	cr->set_dash(dashes, 0);
	*/
				cr->rectangle(line_if_->world_x() - radius * 0.5,
				              line_if_->world_y() - radius * 0.5,
				              radius,
				              radius);
				cr->rectangle(line_if_->relative_x() - radius * 0.5,
				              line_if_->relative_y() - radius * 0.5,
				              radius,
				              radius);
				cr->fill_preserve();
				cr->stroke();
				cr->move_to(line_if_->world_x(), line_if_->world_y());
				cr->line_to(line_if_->relative_x(), line_if_->relative_y());
				cr->stroke();
			}
		}
		cr->restore();
	}

	return true;
}

/** Draw scale box.
 * Draws a circle with a radius of 1m around the robot.
 * @param window Gdk window
 * @param cr Cairo context to draw to. It is assumed that possible transformations
 * have been setup before.
 */
void
LaserDrawingArea::draw_scalebox(Glib::RefPtr<Gdk::Window>           &window,
                                const Cairo::RefPtr<Cairo::Context> &cr)
{
	cr->save();
	cr->set_source_rgba(0, 0, 0.8, 0.2);
	cr->arc(0, 0, 1.0, 0, 2 * M_PI);
	cr->stroke();
	cr->restore();
}

/** Draw Beams of an interface.
 * Draws the beams as lines, circles or hull, depending on draw mode.
 * @param itf either Laser360Interface or Laser720Interface
 * @param window Gdk window
 * @param cr Cairo context to draw to. It is assumed that possible transformations
 * have been setup before.
 */
void
LaserDrawingArea::draw_beams(const fawkes::Interface             *itf,
                             Glib::RefPtr<Gdk::Window>           &window,
                             const Cairo::RefPtr<Cairo::Context> &cr)
{
	float                            *distances;
	size_t                            nd;
	bool                              clockwise;
	const fawkes::Laser360Interface  *itf360  = NULL;
	const fawkes::Laser720Interface  *itf720  = NULL;
	const fawkes::Laser1080Interface *itf1080 = NULL;
	if ((itf360 = dynamic_cast<const fawkes::Laser360Interface *>(itf))) {
		distances = itf360->distances();
		nd        = itf360->maxlenof_distances();
		clockwise = itf360->is_clockwise_angle();
	} else if ((itf720 = dynamic_cast<const fawkes::Laser720Interface *>(itf))) {
		distances = itf720->distances();
		nd        = itf720->maxlenof_distances();
		clockwise = itf720->is_clockwise_angle();
	} else if ((itf1080 = dynamic_cast<const fawkes::Laser1080Interface *>(itf))) {
		distances = itf1080->distances();
		nd        = itf1080->maxlenof_distances();
		clockwise = itf1080->is_clockwise_angle();
	} else {
		throw fawkes::Exception("Interface is neither Laser360Interface nor Laser720Interface");
	}

	const float nd_factor = 360.0 / nd;

	float *revdists = NULL;
	if (!clockwise) {
		// re-arrange to clockwise
		revdists = (float *)new float[nd];
		for (size_t i = 0; i < nd; ++i) {
			revdists[nd - i - 1] = distances[i];
		}
		distances = revdists;
	}

	cr->scale(zoom_factor_, zoom_factor_);
	cr->rotate(rotation_);
	cr->set_line_width(1. / zoom_factor_);

	draw_scalebox(window, cr);

	if (draw_mode_ == MODE_LINES) {
		for (size_t i = 0; i < nd; i += resolution_) {
			if (distances[i] == 0 || !std::isfinite(distances[i]))
				continue;
			const float anglerad = deg2rad(i * nd_factor);
			cr->move_to(0, 0);
			cr->line_to(distances[i] * sin(anglerad), distances[i] * -cos(anglerad));
		}
		cr->stroke();
	} else if (draw_mode_ == MODE_POINTS) {
		const float radius = 4 / zoom_factor_;
		for (size_t i = 0; i < nd; i += resolution_) {
			if (distances[i] == 0)
				continue;
			float anglerad = deg2rad(i * nd_factor);
			float x        = distances[i] * sin(anglerad);
			float y        = distances[i] * -cos(anglerad);
			// circles replaced by rectangles, they are a *lot* faster
			//cr->move_to(x, y);
			//cr->arc(x, y, radius, 0, 2*M_PI);
			cr->rectangle(x, y, radius, radius);
		}
		cr->fill_preserve();
		cr->stroke();
	} else {
		cr->move_to(0, -distances[0]);
		for (size_t i = resolution_; i <= nd + resolution_; i += resolution_) {
			if (distances[i] == 0)
				continue;
			const float anglerad = normalize_rad(deg2rad(i * nd_factor));
			cr->line_to(distances[i % nd] * sin(anglerad), distances[i % nd] * -cos(anglerad));
		}
		cr->stroke();
	}

	if (revdists)
		delete[] revdists;
}

/** Draw person legs.
 * Draws the legs of persons
 * @param window Gdk window
 * @param cr Cairo context to draw to. It is assumed that possible transformations
 * have been setup before.
 */
void
LaserDrawingArea::draw_persons_legs(Glib::RefPtr<Gdk::Window>           &window,
                                    const Cairo::RefPtr<Cairo::Context> &cr)
{
	std::list<ObjectPositionInterface *>::iterator objpos_if_itt;
	;

	cr->save();
	if (l_objpos_if_persons_) {
		cr->set_source_rgb(0, 0, 1);
		for (objpos_if_itt = l_objpos_if_persons_->begin();
		     objpos_if_itt != l_objpos_if_persons_->end() && (*objpos_if_itt)->has_writer();
		     ++objpos_if_itt) {
			if (!break_drawing_)
				(*objpos_if_itt)->read();
			if ((*objpos_if_itt)->is_valid()) {
				std::pair<float, float> pos = transform_coords_from_fawkes((*objpos_if_itt)->relative_x(),
				                                                           (*objpos_if_itt)->relative_y());
				float                   x   = pos.first;
				float                   y   = pos.second;
				cr->move_to(x, y);
				//      cr->arc(x, y, std::max((*objpos_if_itt)->extent_x(),(*objpos_if_itt)->extent_y()), 0, 2*M_PI);
				cr->arc(x, y, 0.2, 0, 2 * M_PI);
			}
		}
		cr->stroke();
	}

	if (l_objpos_if_legs_) {
		cr->set_source_rgb(0, 1, 0);
		for (objpos_if_itt = l_objpos_if_legs_->begin();
		     objpos_if_itt != l_objpos_if_legs_->end() && (*objpos_if_itt)->has_writer();
		     ++objpos_if_itt) {
			if (!break_drawing_) {
				(*objpos_if_itt)->read();
			}
			if ((*objpos_if_itt)->is_valid()) {
				std::pair<float, float> pos = transform_coords_from_fawkes((*objpos_if_itt)->relative_x(),
				                                                           (*objpos_if_itt)->relative_y());
				float                   x   = pos.first;
				float                   y   = pos.second;
				cr->move_to(x, y);
				cr->arc(x, y, 0.1, 0, 2 * M_PI);
			}
		}
		cr->stroke();
	}

	if (l_objpos_if_misc_) {
		cr->set_source_rgb(0, 1, 1);
		for (objpos_if_itt = l_objpos_if_misc_->begin();
		     objpos_if_itt != l_objpos_if_misc_->end() && (*objpos_if_itt)->has_writer();
		     ++objpos_if_itt) {
			if (!break_drawing_)
				(*objpos_if_itt)->read();
			if ((*objpos_if_itt)->is_valid()) {
				//      switch( (*objpos_if_itt)->object_type() ){
				//      case ObjectPositionInterface::TYPE_BALL:
				//TYPE_OPPONENT
				if ((*objpos_if_itt)->object_type() == ObjectPositionInterface::TYPE_BALL) {
					std::pair<float, float> pos =
					  transform_coords_from_fawkes((*objpos_if_itt)->relative_x(),
					                               (*objpos_if_itt)->relative_y());
					float x = pos.first;
					float y = pos.second;
					pos =
					  transform_coords_from_fawkes((*objpos_if_itt)->world_x(), (*objpos_if_itt)->world_y());
					float begin_x     = pos.first;
					float begin_y     = pos.second;
					pos               = transform_coords_from_fawkes((*objpos_if_itt)->world_x_velocity(),
                                             (*objpos_if_itt)->world_y_velocity());
					float end_x       = pos.first;
					float end_y       = pos.first;
					float angle1      = atan2(begin_y - y, begin_x - x);
					float angle2      = atan2(end_y - y, end_x - x);
					float radius      = (*objpos_if_itt)->relative_x_velocity();
					float probability = (*objpos_if_itt)->relative_z_velocity();
					cr->move_to(begin_x, begin_y);
					cr->arc(x, y, radius, angle2, angle1);

					//	Cairo::TextExtents te;
					std::string t = StringConversions::to_string(probability);
					t.erase(5);
					//	  cr->set_source_rgb(0,1 ,1);
					cr->set_font_size(0.08);
					//	cr->get_text_extents(t, te);
					//	cr->move_to(- te.width / 2, -te.height / 2);
					cr->move_to(begin_x, begin_y);
					cr->show_text(t);
					//	  cr->set_source_rgb(0,0,1);

					//	break;
					//      case ObjectPositionInterface::TYPE_LINE:
				} else if ((*objpos_if_itt)->object_type() == ObjectPositionInterface::TYPE_LINE) {
					std::pair<float, float> pos =
					  transform_coords_from_fawkes((*objpos_if_itt)->world_x(), (*objpos_if_itt)->world_y());
					float begin_x = pos.first;
					float begin_y = pos.second;
					pos           = transform_coords_from_fawkes((*objpos_if_itt)->world_x_velocity(),
                                             (*objpos_if_itt)->world_y_velocity());
					float end_x   = pos.first;
					float end_y   = pos.first;
					cr->move_to(begin_x, begin_y);
					cr->line_to(end_x, end_y);
					//break;
				}
			}
		}
		//  cr->fill_preserve();
		cr->stroke();
	}

	cr->set_source_rgb(1, 0, 1);

	float r, g, b;
	r = g = b = 0.0;

	if (l_track_if_) {
		std::list<Position2DTrackInterface *>::iterator track_if_itt;
		;
		const float  radius(0.1);
		float       *x_positions1;
		float       *y_positions1;
		int         *timestamps1;
		float       *x_positions2  = NULL;
		float       *y_positions2  = NULL;
		unsigned int track_length1 = 0;
		unsigned int track_length2 = 0;
		int         *timestamps2   = NULL;
		unsigned int id;
		int          color_it = 0;
		float        delta    = 0.25;
		cr->set_font_size(0.03);
#ifdef LASERGUI_DEBUG_PRINT_TRACKS
		printf("\n\n################################\n");
#endif
		for (track_if_itt = l_track_if_->begin();
		     track_if_itt != l_track_if_->end() && (*track_if_itt)->has_writer();) {
			if (!break_drawing_) {
				(*track_if_itt)->read();
			}
			if ((*track_if_itt)->is_valid()) {
				bool b_compound_track = false;
				x_positions1          = (*track_if_itt)->track_x_positions();
				y_positions1          = (*track_if_itt)->track_y_positions();
				timestamps1           = (*track_if_itt)->track_timestamps();
				track_length1         = (*track_if_itt)->length();
				id                    = (*track_if_itt)->track_id();
				++track_if_itt;
				if (track_if_itt != l_track_if_->end() && (*track_if_itt)->has_writer()) {
					if (!break_drawing_)
						(*track_if_itt)->read();
					if ((*track_if_itt)->is_valid() && (*track_if_itt)->track_id() == id) {
						b_compound_track = true;
						x_positions2     = (*track_if_itt)->track_x_positions();
						y_positions2     = (*track_if_itt)->track_y_positions();
						timestamps2      = (*track_if_itt)->track_timestamps();
						track_length2    = (*track_if_itt)->length();
						++track_if_itt;
					}
				}
#ifdef LASERGUI_DEBUG_PRINT_TRACKS
				printf("\n    trackid %u\n", id);
#endif
				unsigned int i(0);
				unsigned int j(0);
				float        x = x_positions1[i];
				float        y = y_positions1[i];
				if (b_compound_track) {
					while (j + 1 < track_length2 && timestamps2[j] < timestamps1[i]) {
						++j;
					}
					if (timestamps2[j] == timestamps1[i]) {
						x += x_positions2[i];
						x /= 2;
						y += y_positions2[i];
						y /= 2;
					}
				}
				std::pair<float, float> pos = transform_coords_from_fawkes(x, y);
				cr->move_to(pos.first, pos.second);
				for (; i < track_length1; ++i) {
					x = x_positions1[i];
					y = y_positions1[i];
					if (b_compound_track) {
						while (j + 1 < track_length2 && timestamps2[j] < timestamps1[i]) {
							++j;
						}
						if (timestamps2[j] == timestamps1[i]) {
							x += x_positions2[i];
							x /= 2;
							y += y_positions2[i];
							y /= 2;
						}
					}
					std::pair<float, float> pos = transform_coords_from_fawkes(x, y);
					//cr->move_to(pos.first - radius, pos.second);
					//	  cr->arc(pos.first, pos.second, radius, 0, 2*M_PI);
					cr->line_to(pos.first, pos.second);
					//	cr->rectangle(x_positions[i], y_positions[i], 4 / zoom_factor_, 4 / zoom_factor_);

					//	std::string t = StringConversions::toString(id) + "-" + StringConversions::toString(timestamps[i]);
					std::string t = StringConversions::to_string(timestamps1[i]);
					//      cr->move_to(begin_x, begin_y);
					cr->show_text(t);
					cr->move_to(pos.first, pos.second);
#ifdef LASERGUI_DEBUG_PRINT_TRACKS
					printf("( %f,%f,[%d] )", pos.first, pos.second, timestamps1[i]);
#endif
				}

				// chose color
				if (div(color_it, 3).rem == 0)
					r += delta;
				if (div(color_it, 3).rem == 1)
					g += delta;
				if (div(color_it, 3).rem == 2)
					b += delta;
				cr->set_source_rgb(r, g, b);
				color_it++;

				cr->stroke();

				i = std::max(0, (int)track_length1 - CFG_PRINT_NR_TRACKELEMENTS);
				j = 0;
				for (; i < track_length1; ++i) {
					x = x_positions1[i];
					y = y_positions1[i];
					if (b_compound_track) {
						while (j + 1 < track_length2 && timestamps2[j] < timestamps1[i]) {
							++j;
						}
					}

					std::pair<float, float> pos =
					  transform_coords_from_fawkes(x_positions1[i], y_positions1[i]);
					cr->move_to(pos.first - radius, pos.second);
					cr->arc(pos.first, pos.second, radius, 0, 2 * M_PI);

					if (b_compound_track && timestamps2[j] == timestamps1[i]) {
						cr->move_to(pos.first, pos.second);

						std::pair<float, float> pos =
						  transform_coords_from_fawkes(x_positions2[j], y_positions2[j]);
						cr->line_to(pos.first, pos.second);
						cr->move_to(pos.first - radius, pos.second);
						cr->arc(pos.first, pos.second, radius, 0, 2 * M_PI);
					}
				}
				cr->set_source_rgb(0, 0, 1);
				cr->stroke();

			} else {
				break;
			}
		}
	}

	/*  DRAW TARGET */
	if (target_if_ && target_if_->has_writer()) {
		target_if_->read();
		if (target_if_->is_valid()) {
			cr->set_source_rgb(1, 0, 0);
			std::pair<float, float> pos =
			  transform_coords_from_fawkes(target_if_->relative_x(), target_if_->relative_y());
			float x      = pos.first;
			float y      = pos.second;
			float radius = 0.1;

			cr->move_to(x, y);
			cr->arc(x, y, radius, 0, 2 * M_PI);
			cr->move_to(x - radius, y);
			cr->line_to(x + radius, y);
			cr->move_to(x, y - radius);
			cr->line_to(x, y + radius);
			cr->stroke();
		}
	}

	/*
  float r,g,b;
  r=g=b=0.0;
  float delta = 0.2;
  for (int i = 0; i< 15 ; i++){

    if (div(i,3).rem == 0) r+= delta;
    if (div(i,3).rem == 1) g+= delta;
    if (div(i,3).rem == 2) b+= delta;
    //    printf("i %d rem %d| r %f, g %f, b %f\n", i, div(i,3).rem,r,g,b);
    cr->move_to(0, (i+1)*0.125);
    cr->set_source_rgb(r,g,b);
    cr->rectangle(0, (i+1)*0.125, 0.1 , 0.1 );
    cr->fill_preserve();
    cr->stroke();
  }
  */
	//  cr->stroke();

	cr->restore();
}

/** Draw laser segments as produced by leg tracker application.
 * @param itf either Laser360Interface or Laser720Interface
 * @param window Gdk window
 * @param cr Cairo context to draw to. It is assumed that possible transformations
 * have been setup before.
 */
void
LaserDrawingArea::draw_segments(const fawkes::Interface             *itf,
                                Glib::RefPtr<Gdk::Window>           &window,
                                const Cairo::RefPtr<Cairo::Context> &cr)
{
	size_t      nd        = laser_segmentation_if_->maxlenof_distances();
	const float nd_factor = 360.0 / nd;

	float                            *distances;
	const fawkes::Laser360Interface  *itf360  = NULL;
	const fawkes::Laser720Interface  *itf720  = NULL;
	const fawkes::Laser1080Interface *itf1080 = NULL;
	if ((itf360 = dynamic_cast<const fawkes::Laser360Interface *>(itf))) {
		distances = itf360->distances();
	} else if ((itf720 = dynamic_cast<const fawkes::Laser720Interface *>(itf))) {
		distances = itf720->distances();
	} else if ((itf1080 = dynamic_cast<const fawkes::Laser1080Interface *>(itf))) {
		distances = itf1080->distances();
	} else {
		throw fawkes::Exception("Interface is neither Laser360Interface nor Laser720Interface");
	}

	cr->save();
	/* DRAW SEGMENTS (draw the segment interiors again with other color*/
	if (laser_segmentation_if_ && laser_segmentation_if_->has_writer()) {
		if (!break_drawing_)
			laser_segmentation_if_->read();
		float *segmentations = laser_segmentation_if_->distances();
		size_t nd            = laser_segmentation_if_->maxlenof_distances();
		//	cr->set_source_rgba(0,0,0,0.5);
		cr->set_source_rgb(1, 1, 0);

		if (draw_mode_ == MODE_POINTS) {
			for (size_t i = 0; i < nd; i += resolution_) {
				if (segmentations[i] == 0)
					continue; // dont draw the segment borders
				if (distances[i] == 0 || !std::isfinite(distances[i]))
					continue;
				float anglerad = deg2rad(i * nd_factor);
				cr->move_to(0, 0);
				cr->line_to(distances[i] * sin(anglerad), distances[i] * -cos(anglerad));
			}
			cr->stroke();
		} else { //if ( draw_mode_ == MODE_LINES ) {
			float radius = 4 / zoom_factor_;
			for (size_t i = 0; i < nd; i += resolution_) {
				if (segmentations[i] == 0)
					continue; // dont draw the segment borders
				if (distances[i] == 0)
					continue;
				float anglerad = deg2rad(i * nd_factor);
				float x        = distances[i] * sin(anglerad);
				float y        = distances[i] * -cos(anglerad);
				// circles replaced by rectangles, they are a *lot* faster
				//cr->move_to(x, y);
				//cr->arc(x, y, radius, 0, 2*M_PI);
				cr->rectangle(x, y, radius, radius);
			}
			cr->fill_preserve();
			cr->stroke();
		}
		/*else {
      cr->move_to(0, - distances[0]);
      for (size_t i = resolution_; i <= nd + resolution_; i += resolution_) {
      if ( distances[i] == 0 )  continue;
      float anglerad    = deg2rad(i % 360);
      cr->line_to(distances[i % 360] *  sin(anglerad),
      distances[i % 360] * -cos(anglerad));
      }
      cr->stroke();
      }
    */
	}
	cr->restore();
}

/** Scroll event handler.
 * @param event event structure
 * @return signal return value
 */
bool
LaserDrawingArea::on_scroll_event(GdkEventScroll *event)
{
	if (event->direction == GDK_SCROLL_UP) {
		zoom_in();
	} else if (event->direction == GDK_SCROLL_DOWN) {
		zoom_out();
	}
	return true;
}

/** Set a member for breaking the drawing. */
void
LaserDrawingArea::toggle_break_drawing()
{
	break_drawing_ = !break_drawing_;
}

/** Button press event handler.
 * @param event event data
 * @return true
 */
bool
LaserDrawingArea::on_button_press_event(GdkEventButton *event)
{
	last_mouse_x_ = event->x;
	last_mouse_y_ = event->y;

	double                        user_x = event->x;
	double                        user_y = event->y;
	Glib::RefPtr<Gdk::Window>     window = get_window();
	Cairo::RefPtr<Cairo::Context> cr     = window->create_cairo_context();
	cr->save();
	cr->translate(xc_, yc_);
	cr->rotate(0.5 * M_PI + rotation_);
	cr->scale(-zoom_factor_, zoom_factor_);
	cr->device_to_user(user_x, user_y);
	printf("Clicked at (%.3lf, %.3lf)\n", user_x, user_y);
	cr->restore();
	return true;
}

/** Mouse motion notify event handler.
 * @param event event data
 * @return true
 */
bool
LaserDrawingArea::on_motion_notify_event(GdkEventMotion *event)
{
	//  dtranslation_x_ -= last_mouse_x_ - event->x;
	//  double translation_y_ -= last_mouse_y_ - event->y;
	xc_ -= last_mouse_x_ - event->x;
	yc_ -= last_mouse_y_ - event->y;

	last_mouse_x_ = event->x;
	last_mouse_y_ = event->y;
	queue_draw();
	return true;
}

/**
 * Transform a position from the fawkes coordinate system to the Cairo
 * coordinate system.
 * @param p_x input x
 * @param p_y input y
 * @return the transformed position
 */
std::pair<float, float>
LaserDrawingArea::transform_coords_from_fawkes(float p_x, float p_y)
{
	std::pair<float, float> pos;
	pos.first  = -p_y;
	pos.second = -p_x;
	return pos;
}
