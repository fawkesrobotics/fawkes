
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

#define M_2xPI (2*M_PI)
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
YuvViewerGtkWindow::YuvViewerGtkWindow(BaseObjectType* cobject,
				       const Glib::RefPtr<Gtk::Builder> builder)
  : Gtk::Window(cobject)
{
  builder->get_widget("yuv_vp",  __yuv_vp);
  builder->get_widget("cur_vp",  __cur_vp);
  builder->get_widget("seg_vp",  __seg_vp);
  builder->get_widget("y_scale", __y_scale);
  builder->get_widget("u_value", __u_value);
  builder->get_widget("v_value", __v_value);
  builder->get_widget("y_res",   __y_res);
  builder->get_widget("u_res",   __u_res);
  builder->get_widget("v_res",   __v_res);

  __yuv_widget = Gtk::manage(new ImageWidget(256, 256));
  __cur_widget = Gtk::manage(new ImageWidget( 60, 40));
  __seg_widget = Gtk::manage(new ImageWidget(256, 256));

  __y_scale->signal_value_changed().connect(sigc::mem_fun(*this, &YuvViewerGtkWindow::on_y_value_changed));
  __y_res->signal_value_changed().connect(sigc::mem_fun(*this, &YuvViewerGtkWindow::on_y_res_changed));
  __u_res->signal_value_changed().connect(sigc::mem_fun(*this, &YuvViewerGtkWindow::on_uv_res_changed));
  __v_res->signal_value_changed().connect(sigc::mem_fun(*this, &YuvViewerGtkWindow::on_uv_res_changed));

  __yuv_vp->signal_button_press_event().connect(sigc::mem_fun(*this, &YuvViewerGtkWindow::on_click_on_yuv));
  __yuv_vp->signal_motion_notify_event().connect(sigc::mem_fun(*this, &YuvViewerGtkWindow::on_mouse_over_yuv));
  __yuv_vp->add(*__yuv_widget);
  __cur_vp->add(*__cur_widget);
  __seg_vp->add(*__seg_widget);


  memset(__cur_buffer + 60 * 40, 128,  60 * 40);
  memset(__seg_buffer, 128, 256 * 256);
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

  __u_value->set_text(convert_float2str(u, 0));
  __v_value->set_text(convert_float2str(v, 0));
  memset(__cur_buffer + 60 * 40, u,  60 * 20);
  memset(__cur_buffer + 60 * 60, v,  60 * 20);
  __cur_widget->show(YUV422_PLANAR, __cur_buffer);

  return true;
}

/** Signal handler called when the Y value changes (HSlider) */
void
YuvViewerGtkWindow::on_y_value_changed()
{
  unsigned int y = round(__y_scale->get_value());
  memset(__yuv_buffer, y, 256 * 256);
  memset(__cur_buffer, y, 60 * 40);

  Drawer d;
  d.set_buffer(__yuv_buffer, 256, 256);
  d.set_color(YUV_t::black());
  d.draw_line(127, 127, 0, 64);
  d.draw_line(127, 127, 64, 0);

  d.draw_line(128, 127, 192, 0);
  d.draw_line(128, 127, 255, 64);

  d.draw_line(128, 128, 192, 255);
  d.draw_line(128, 128, 255, 192);

  d.draw_line(127, 128, 0, 192);
  d.draw_line(127, 128, 64, 255);

  __yuv_widget->show(YUV422_PLANAR, __yuv_buffer);
  __cur_widget->show(YUV422_PLANAR, __cur_buffer);
}


void
YuvViewerGtkWindow::on_y_res_changed()
{
  unsigned int r = round(__y_res->get_value());

  if (r == 0) {
    __y_scale->set_value(127);
    __y_scale->set_range(127, 128);
  }
  else {
    __y_scale->set_range(0, 255);
    __y_scale->set_increments(255.f / (pow(2, r) - 1), 0);
  }
}

void
YuvViewerGtkWindow::on_uv_res_changed()
{
  unsigned char *yuv_u = __yuv_buffer + 256 * 256;
  unsigned char *yuv_v = yuv_u + 256 * 256 / 2;
  unsigned int u_div = 256 / (int)pow(2, __u_res->get_value());
  unsigned int v_div = 256 / (int)pow(2, __v_res->get_value());

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
#if GLIBMM_MAJOR_VERSION > 2 || ( GLIBMM_MAJOR_VERSION == 2 && GLIBMM_MINOR_VERSION >= 16 )
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
  YUV_t c;
  unsigned char *seg_u = __seg_buffer + 256 * 256;
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
        if (dv > 0) YUV_t::red();
        else c = YUV_t::gray();
      }
      else {
        float a = atan2f(dv, du);
        if (a < 0) a += M_2xPI;

        if (a >= a1 && a < a2) c = YUV_t::magenta();
        else if (a >= a2 && a < a3) c = YUV_t::red();
        else if (a >= a3 && a < a4) c = YUV_t::orange();
        else if (a >= a4 && a < a5) c = YUV_t::yellow();
        else if (a >= a5 && a < a6) c = YUV_t::green();
        else if (a >= a6 && a < a7) c = YUV_t::gray();
        else if (a >= a7 && a < a8) c = YUV_t::cyan();
        else c = YUV_t::blue();
      }

      unsigned int addr = ((255 - v) * 256 + u) / 2;
      seg_u[addr] = c.U;
      seg_v[addr] = c.V;
    }
  }

  __seg_widget->show(YUV422_PLANAR, __seg_buffer);
}
