
/***************************************************************************
 *  yuv_viewer.h - YUV viewer gui
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

#ifndef _FIREVISION_TOOLS_YUV_VIEWER_LOC_VIEWER_GUI_H_
#define _FIREVISION_TOOLS_YUV_VIEWER_LOC_VIEWER_GUI_H_

#define LOC_PLUGIN_NAME "fvnao_loc"
#define FUSE_PLUGIN_NAME "fvfountain"
#define FOUNTAIN_PORT_PATH "/firevision/fountain/tcp_port"

#include <fvutils/color/yuv.h>
#include <fvwidgets/image_widget.h>

#include <gtkmm.h>

using namespace firevision;

class YuvViewerGtkWindow : public Gtk::Window
{
private:
public:
	YuvViewerGtkWindow(BaseObjectType *cobject, const Glib::RefPtr<Gtk::Builder> builder);
	virtual ~YuvViewerGtkWindow();

private:
	bool          on_mouse_over_yuv(GdkEventMotion *event);
	bool          on_click_on_yuv(GdkEventButton *event);
	void          on_y_value_changed();
	void          on_y_res_changed();
	void          on_uv_res_changed();
	void          calc_seg();
	Glib::ustring convert_float2str(float f, unsigned int width);

private:
	// widgets
	Gtk::EventBox *  yuv_vp_;
	Gtk::Viewport *  cur_vp_;
	Gtk::Viewport *  seg_vp_;
	Gtk::HScale *    y_scale_;
	Gtk::Label *     u_value_;
	Gtk::Label *     v_value_;
	Gtk::SpinButton *y_res_;
	Gtk::SpinButton *u_res_;
	Gtk::SpinButton *v_res_;

	ImageWidget *yuv_widget_;
	ImageWidget *cur_widget_;
	ImageWidget *seg_widget_;

	unsigned char yuv_buffer_[256 * 256 * 2];
	unsigned char cur_buffer_[60 * 40 * 2];
	unsigned char seg_buffer_[256 * 256 * 2];
};

#endif /* FIREVISION_TOOLS_YUV_VIEWER_LOC_VIEWER_GUI_H__ */
