
/***************************************************************************
 *  yuv_viewer.cpp - YUV viewer gui
 *
 *  Created:  Sat Mar 22 16:34:02 2009
 *  Copyright 2009 Christof Rath <c.rath@student.tugraz.at>
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

#include "yuv_viewer_gui.h"

#include <fvutils/color/colorspaces.h>
#include <fvutils/draw/drawer.h>

#include <cmath>
#include <cstring>
#include <iomanip>
#include <sstream>

#define M_2xPI (2 * M_PI)
using namespace fawkes;
using namespace firevision;

/** @class YuvViewerGtkWindow "yuv_viewer_gui.h"
 * Tool to show the YUV color space
 *
 * @author Christof Rath
 */

/** Constructor.
 * @param cobject C base object
 * @param builder Gtk builder
 */
YuvViewerGtkWindow::YuvViewerGtkWindow(BaseObjectType *                 cobject,
                                       const Glib::RefPtr<Gtk::Builder> builder)
: Gtk::Window(cobject)
{
	builder->get_widget("yuv_vp", yuv_vp_);
	builder->get_widget("cur_vp", cur_vp_);
	builder->get_widget("seg_vp", seg_vp_);
	builder->get_widget("y_scale", y_scale_);
	builder->get_widget("u_value", u_value_);
	builder->get_widget("v_value", v_value_);
	builder->get_widget("y_res", y_res_);
	builder->get_widget("u_res", u_res_);
	builder->get_widget("v_res", v_res_);

	yuv_widget_ = Gtk::manage(new ImageWidget(256, 256));
	cur_widget_ = Gtk::manage(new ImageWidget(60, 40));
	seg_widget_ = Gtk::manage(new ImageWidget(256, 256));

	y_scale_->signal_value_changed().connect(
	  sigc::mem_fun(*this, &YuvViewerGtkWindow::on_y_value_changed));
	y_res_->signal_value_changed().connect(
	  sigc::mem_fun(*this, &YuvViewerGtkWindow::on_y_res_changed));
	u_res_->signal_value_changed().connect(
	  sigc::mem_fun(*this, &YuvViewerGtkWindow::on_uv_res_changed));
	v_res_->signal_value_changed().connect(
	  sigc::mem_fun(*this, &YuvViewerGtkWindow::on_uv_res_changed));

	yuv_vp_->signal_button_press_event().connect(
	  sigc::mem_fun(*this, &YuvViewerGtkWindow::on_click_on_yuv));
	yuv_vp_->signal_motion_notify_event().connect(
	  sigc::mem_fun(*this, &YuvViewerGtkWindow::on_mouse_over_yuv));
	yuv_vp_->add(*yuv_widget_);
	cur_vp_->add(*cur_widget_);
	seg_vp_->add(*seg_widget_);

	memset(cur_buffer_ + 60 * 40, 128, 60 * 40);
	memset(seg_buffer_, 128, 256 * 256);
	on_y_value_changed();
	on_uv_res_changed();
	calc_seg();
	show_all_children();
}

/** Destructor. */
YuvViewerGtkWindow::~YuvViewerGtkWindow()
{
}

/** Signal hander that gets called after a click on the YUV pane
 * @param event provides the x/y-coordinate
 * @return true
 */
bool
YuvViewerGtkWindow::on_click_on_yuv(GdkEventButton *event)
{
	GdkEventMotion mot;
	mot.x = event->x;
	mot.y = event->y;
	return on_mouse_over_yuv(&mot);
}

/** Signal hander that gets called during a movement on the YUV pane (if the left button is pressed)
 * @param event provides the x/y-coordinate
 * @return true
 */
bool
YuvViewerGtkWindow::on_mouse_over_yuv(GdkEventMotion *event)
{
	unsigned int u = std::max(0, std::min(255, (int)event->x));
	unsigned int v = 255 - std::max(0, std::min(255, (int)event->y));

	u_value_->set_text(convert_float2str(u, 0));
	v_value_->set_text(convert_float2str(v, 0));
	memset(cur_buffer_ + 60 * 40, u, 60 * 20);
	memset(cur_buffer_ + 60 * 60, v, 60 * 20);
	cur_widget_->show(YUV422_PLANAR, cur_buffer_);

	return true;
}

/** Signal handler called when the Y value changes (HSlider) */
void
YuvViewerGtkWindow::on_y_value_changed()
{
	unsigned int y = round(y_scale_->get_value());
	memset(yuv_buffer_, y, 256 * 256);
	memset(cur_buffer_, y, 60 * 40);

	Drawer d;
	d.set_buffer(yuv_buffer_, 256, 256);
	d.set_color(YUV_t::black());
	d.draw_line(127, 127, 0, 64);
	d.draw_line(127, 127, 64, 0);

	d.draw_line(128, 127, 192, 0);
	d.draw_line(128, 127, 255, 64);

	d.draw_line(128, 128, 192, 255);
	d.draw_line(128, 128, 255, 192);

	d.draw_line(127, 128, 0, 192);
	d.draw_line(127, 128, 64, 255);

	yuv_widget_->show(YUV422_PLANAR, yuv_buffer_);
	cur_widget_->show(YUV422_PLANAR, cur_buffer_);
}

void
YuvViewerGtkWindow::on_y_res_changed()
{
	unsigned int r = round(y_res_->get_value());

	if (r == 0) {
		y_scale_->set_value(127);
		y_scale_->set_range(127, 128);
	} else {
		y_scale_->set_range(0, 255);
		y_scale_->set_increments(255.f / (pow(2, r) - 1), 0);
	}
}

void
YuvViewerGtkWindow::on_uv_res_changed()
{
	unsigned char *yuv_u = yuv_buffer_ + 256 * 256;
	unsigned char *yuv_v = yuv_u + 256 * 256 / 2;
	unsigned int   u_div = 256 / (int)pow(2, u_res_->get_value());
	unsigned int   v_div = 256 / (int)pow(2, v_res_->get_value());

	for (unsigned int v = 0; v < 256; ++v) {
		memset((yuv_v + v * 128), ((255 - v) / v_div) * v_div, 128);

		for (unsigned int u = 0; u < 128; ++u) {
			yuv_u[v * 128 + u] = (u * 2 / u_div) * u_div;
		}
	}

	on_y_value_changed();
}

/**
 * Converts a float value to a Glib::ustring (locale dependent)
 * @param f The float value
 * @param width The precision width
 * @return the formatted string
 */
Glib::ustring
YuvViewerGtkWindow::convert_float2str(float f, unsigned int width)
{
#if GLIBMM_MAJOR_VERSION > 2 || (GLIBMM_MAJOR_VERSION == 2 && GLIBMM_MINOR_VERSION >= 16)
	return Glib::ustring::format(std::fixed, std::setprecision(width), f);
#else
	std::ostringstream ss;
	ss << std::fixed << std::setprecision(width);
	ss << f;

	return Glib::locale_to_utf8(ss.str());
#endif
}

/** Calculates the segmented window */
void
YuvViewerGtkWindow::calc_seg()
{
	YUV_t          c;
	unsigned char *seg_u = seg_buffer_ + 256 * 256;
	unsigned char *seg_v = seg_u + 256 * 256 / 2;

	float a1 = atan2f(64, 128);
	float a2 = atan2f(128, 64);
	float a3 = atan2f(128, -64);
	float a4 = atan2f(64, -128);
	float a5 = atan2f(-64, -128) + M_2xPI;
	float a6 = atan2f(-128, -64) + M_2xPI;
	float a7 = atan2f(-128, 64) + M_2xPI;
	float a8 = atan2f(-64, 128) + M_2xPI;

	for (int u = 0; u < 256; ++u) {
		float du = u - 128;

		for (int v = 255; v >= 0; --v) {
			float dv = v - 128;

			if (!du) {
				if (dv > 0)
					YUV_t::red();
				else
					c = YUV_t::gray();
			} else {
				float a = atan2f(dv, du);
				if (a < 0)
					a += M_2xPI;

				if (a >= a1 && a < a2)
					c = YUV_t::magenta();
				else if (a >= a2 && a < a3)
					c = YUV_t::red();
				else if (a >= a3 && a < a4)
					c = YUV_t::orange();
				else if (a >= a4 && a < a5)
					c = YUV_t::yellow();
				else if (a >= a5 && a < a6)
					c = YUV_t::green();
				else if (a >= a6 && a < a7)
					c = YUV_t::gray();
				else if (a >= a7 && a < a8)
					c = YUV_t::cyan();
				else
					c = YUV_t::blue();
			}

			unsigned int addr = ((255 - v) * 256 + u) / 2;
			seg_u[addr]       = c.U;
			seg_v[addr]       = c.V;
		}
	}

	seg_widget_->show(YUV422_PLANAR, seg_buffer_);
}
