
/***************************************************************************
 *  color_train.h - Color training tool
 *
 *  Created: Fri Dec 07 18:41:57 2007
 *  Copyright  2007  Daniel Beck
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

#ifndef __FIREVISION_TOOLS_IMAGE_VIEWER_COLOR_TRAIN_H_
#define __FIREVISION_TOOLS_IMAGE_VIEWER_COLOR_TRAIN_H_

#include <fvutils/base/roi.h>
#include <stdlib.h>
#include <gtkmm.h>

class Zauberstab;
class BayesColorLutGenerator;

class ColorTrainTool
{
 public:
  ColorTrainTool();
  ColorTrainTool(unsigned int width, unsigned int height,
		 unsigned char* yuv_buffer, unsigned char* draw_buffer,
		 hint_t fg_object);
  ~ColorTrainTool();

  void step(unsigned int x, unsigned int y);
  void add();
  void unselect();
  void reset();

  void perform_segmentation();

  void save_histos();
  void save_colormap(const char* filename);
  void load_colormap(const char* filename);

  unsigned char* colormap_rgb(unsigned int* width, unsigned int* height);

  void initialize(unsigned int width, unsigned int height,
		  unsigned char* yuv_buffer, unsigned char* draw_buffer,
		  hint_t fg_object);
  void set_src_buffer(unsigned char* buffer, unsigned int width, unsigned int height);
  void set_draw_buffer(unsigned char* buffer);
  void set_fg_object(hint_t object);
  void set_threshold(unsigned int threshold);
  bool set_threshold_sh(Gtk::ScrollType scroll, double value);
  void set_min_prob(float prob);
  bool set_min_prob_sh(Gtk::ScrollType scroll, double value);

  unsigned char* seg_buffer();

  bool initialized();
  bool data_available();

 private:
  BayesColorLutGenerator* m_generator;
  Zauberstab* m_zauberstab;

  unsigned char* m_buffer;
  unsigned char* m_draw_buffer;
  unsigned char* m_seg_buffer;
  unsigned int m_img_width;
  unsigned int m_img_height;
  size_t m_img_size;

  unsigned char* m_lut_rgb_buffer;
  hint_t m_fg_object;
  bool m_initialized;
  bool m_data_available;
};

#endif /* __FIREVISION_TOOLS_IMAGE_VIEWER_COLOR_TRAIN_H_ */
