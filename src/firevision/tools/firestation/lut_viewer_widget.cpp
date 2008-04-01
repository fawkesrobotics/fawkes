
/***************************************************************************
 *  lut_viewer_widget.cpp - Viewer widget for lookup tables
 *
 *  Created: Thu Mar 20 19:08:04 2008
 *  Copyright  2008  Daniel Beck
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

#include <tools/firestation/lut_viewer_widget.h>
#include <models/color/lookuptable.h>
#include <fvutils/scalers/lossy.h>
#include <fvutils/color/conversions.h>

/** @class LutViewerWidget tools/firestation/lut_viewer_widget.h
 * Displays a LUT in an Image.
 *
 * @author Daniel Beck
 */

/** Constructor. */
LutViewerWidget::LutViewerWidget()
{
  m_cm = 0;
  m_img_lut = 0;
  m_lut_img_buf = 0;
}

/** Destructor. */
LutViewerWidget::~LutViewerWidget()
{
  free(m_lut_img_buf);
}

/** Set the colormap to display.
 * @param cm colormap
 */
void
LutViewerWidget::set_colormap(YuvColormap *cm)
{
  m_cm = cm;
}

/** Set the image to render into.
 * @param img the Image
 */
void
LutViewerWidget::set_lut_img(Gtk::Image* img)
{
  m_img_lut = img;
}

/** Draw the LUT. */
void
LutViewerWidget::draw()
{
  if (m_cm == 0 || m_img_lut == 0)
    { return; }

  unsigned char* lut_buffer = (unsigned char*) malloc( colorspace_buffer_size(YUV422_PLANAR, m_cm->width() * 2, m_cm->height() * 2) );
  m_cm->to_image(lut_buffer);

  unsigned int img_width = (unsigned int) m_img_lut->get_width();
  unsigned int img_height = (unsigned int) m_img_lut->get_height();

  img_width = (img_width < img_height) ? img_width : img_height;
  img_height = (img_width < img_height) ? img_width : img_height;

  // scale
  LossyScaler scaler;
  scaler.set_original_buffer(lut_buffer);
  scaler.set_original_dimensions(m_cm->width() * 2, m_cm->height() * 2);
  scaler.set_scaled_dimensions(img_width, img_height);
  unsigned char* scaled_lut_buffer = (unsigned char*) malloc( colorspace_buffer_size(YUV422_PLANAR, img_width, img_height) );
  scaler.set_scaled_buffer(scaled_lut_buffer);
  scaler.scale();

  free(m_lut_img_buf);
  m_lut_img_buf = (unsigned char*) malloc( colorspace_buffer_size(RGB, img_width, img_height) );
  convert(YUV422_PLANAR, RGB, scaled_lut_buffer, m_lut_img_buf, img_width, img_height);

  Glib::RefPtr<Gdk::Pixbuf> lut_image = Gdk::Pixbuf::create_from_data( m_lut_img_buf,
								       Gdk::COLORSPACE_RGB,
								       false,
								       8,
								       img_width, img_height,
								       3 * img_width);
  m_img_lut->set(lut_image);

  free(lut_buffer);
  free(scaled_lut_buffer);
}
