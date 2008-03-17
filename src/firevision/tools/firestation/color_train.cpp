
/***************************************************************************
 *  color_train.cpp - Color training tool
 *
 *  Created: Fri Dec 07 18:47:16 2007
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

#include <tools/firestation/color_train.h>

#include <models/color/bayes/bayes_generator.h>
#include <models/color/lookuptable.h>
#include <fvutils/color/zauberstab.h>
#include <fvutils/color/colorspaces.h>
#include <fvutils/color/conversions.h>
#include <fvutils/color/yuv.h>
#include <fvutils/draw/drawer.h>
#include <fvutils/statistical/histogram.h>
#include <fvutils/scalers/lossy.h>

#include <string.h>

#include <map>

/** @class ColorTrainTool tools/firestation/color_train.h
 * This class encapsulates the routines necessary for interactive color
 * training.
 */

using namespace std;

/** Constructor. */
ColorTrainTool::ColorTrainTool()
{
  m_img_width = 0;
  m_img_height = 0;
  m_img_size = 0;

  m_buffer = 0;
  m_draw_buffer = 0;
  m_seg_buffer = 0;
 
  m_fg_object = H_UNKNOWN;

  m_lut_rgb_buffer = 0;

  m_zauberstab = 0;
  m_generator = 0;

  m_data_available = false;
  m_initialized = false;
}

/** Constructor.
 * @param width image width
 * @param height image height
 * @param yuv_buffer pointer to the image buffer
 * @param draw_buffer pointer to the draw buffer
 * @param fg_object forground object
 */
ColorTrainTool::ColorTrainTool(unsigned int width, unsigned int height,
			       unsigned char* yuv_buffer, unsigned char* draw_buffer,
			       hint_t fg_object)
{
  m_initialized = false;
  initialize(width, height, yuv_buffer, draw_buffer, fg_object);
}

/** Destructor. */
ColorTrainTool::~ColorTrainTool()
{
  free(m_lut_rgb_buffer);
  delete m_zauberstab;
  delete m_generator;
  delete m_seg_buffer;
}

/** Initializer routine.
 * @param width image width
 * @param height image height
 * @param yuv_buffer pointer to the image buffer
 * @param draw_buffer pointer to the draw buffer
 * @param fg_object forground object
 */
void
ColorTrainTool::initialize(unsigned int width, unsigned int height,
			   unsigned char* yuv_buffer, unsigned char* draw_buffer,
			   hint_t fg_object)
{
  if (m_initialized)
    {
      free(m_seg_buffer);
    }

  m_img_width = width;
  m_img_height = height;
  m_img_size = colorspace_buffer_size( YUV422_PLANAR,
				       m_img_width,
				       m_img_height );
  m_buffer = yuv_buffer;
  m_draw_buffer = draw_buffer;
  m_seg_buffer = (unsigned char*) malloc(m_img_size);
  bzero(m_seg_buffer, m_img_size);
  m_fg_object = fg_object;

  m_lut_rgb_buffer = 0;

  if ( !m_zauberstab)
    {
      m_zauberstab = new Zauberstab();
    }
  m_zauberstab->setBuffer(m_buffer, m_img_width, m_img_height);
  m_zauberstab->setThreshold(10);

  if ( !m_generator)
    {
      m_generator = new BayesColorLutGenerator(m_fg_object);
    }

  m_data_available = false;
  m_initialized = true;
}

/** Do one step in the color training process.
 * @param x x-coordinate of the pixel to be evaluated
 * @param y y-coordinate of the pixel to be evaluated
 */
void
ColorTrainTool::step(unsigned int x, unsigned int y)
{
  if ( !m_initialized)
    {
      printf("uninitialized\n");
      return;
    }

  if ( m_zauberstab->isEmptyRegion() )
    {
      printf("finding region around %d,%d\n", x, y);
      m_zauberstab->findRegion(x, y);
    }
  else
    {
      printf("adding region around %d,%d\n", x, y);
      m_zauberstab->addRegion(x, y);
    }
  
  memcpy(m_draw_buffer, m_buffer, m_img_size);

  ZRegion *region = m_zauberstab->getRegion();
  Drawer *d = new Drawer();
  d->setBuffer( m_draw_buffer, m_img_width, m_img_height );	  
  
  for (unsigned int s = 0; s < region->slices->size(); s++) 
    {
      d->drawRectangleInverted( region->slices->at(s)->leftX,
				region->slices->at(s)->y,
				region->slices->at(s)->rightX - region->slices->at(s)->leftX,
				1 ); 
    }
  
  delete d;

  m_data_available = true;
}

/** Add the current selection to the colormap. */
void
ColorTrainTool::add()
{
  if (m_data_available)
    {
      m_generator->reset_undo();
      m_generator->set_buffer(m_buffer, m_img_width, m_img_height);
      m_generator->set_selection( m_zauberstab->getSelection() );
      m_generator->consider();
      m_generator->calc();
      m_data_available = false;
    }
}

/** Cancel the current selction and reset the draw buffer. */
void
ColorTrainTool::unselect()
{
  m_zauberstab->deleteRegion();
  memcpy(m_draw_buffer, m_buffer, m_img_size);
  m_data_available = false;
}

/** Reset the selction, the histograms, and the colormap. */
void
ColorTrainTool::reset()
{
  m_zauberstab->deleteRegion();
  m_generator->reset();
  memcpy(m_draw_buffer, m_buffer, m_img_size);
  m_data_available = false;
}

/** Determines the most likely object for every pixel and marks it
 * accordingly. 
 */
void
ColorTrainTool::perform_segmentation()
{
  if ( !m_initialized )
    {
      printf("perform_segmentation: not initialized\n");
      return;
    }

  bzero(m_seg_buffer, m_img_size);

  Drawer* d = new Drawer();
  d->setBuffer(m_seg_buffer, m_img_width, m_img_height);
  
  ColorModelLookupTable* cm = m_generator->get_current();
  
  for (unsigned int w = 0; w < m_img_width; ++w)
    {
      for (unsigned int h = 0; h < m_img_height; ++h)
	{
	  color_t result;
	  unsigned int y = YUV422_PLANAR_Y_AT(m_buffer, m_img_width, w, h);
	  unsigned int u = YUV422_PLANAR_U_AT(m_buffer, m_img_width, m_img_height, w, h);
	  unsigned int v = YUV422_PLANAR_V_AT(m_buffer, m_img_width, m_img_height, w, h);
	  result = cm->determine(y, u, v);				  
	  
	  if (C_GREEN == result)
	    {
	      d->setColor(240, 0, 0);
	      d->colorPoint(w, h);
	    }
	  else if (C_ORANGE == result)
	    {
	      d->setColor(160, 0, 255);
	      d->colorPoint(w, h);
	    }
	  else
	    {
	      d->setColor(127, 127, 127);
	      d->colorPoint(w, h);
	    }
	}
    }
}

/** Save histograms to disk. */
void
ColorTrainTool::save_histos()
{
  std::map<hint_t, Histogram*>* histos = m_generator->get_histograms();
  std::map<hint_t, Histogram*>::iterator hit;
  for (hit = histos->begin(); hit != histos->end(); ++hit)
    {
      char* filename;
      switch(hit->first)
	{
	case H_BALL:
	  filename = strdup("ball.histo");
	  break;

	case H_BACKGROUND:
	  filename = strdup("background.histo");
	  break;

	case H_FIELD:
	  filename = strdup("field.histo");
	  break;

	default:
	  filename = strdup("asdf.histo");
	}
      hit->second->save(filename, true);
      free(filename);
    }
}

/** Save colormap to a file.
 * @param filename the name of the file
 */
void
ColorTrainTool::save_colormap(const char* filename)
{
  if (m_generator)
    {
      ColorModelLookupTable* lut = m_generator->get_current();
      lut->save(filename);
    }
}

/** Load a colormap from a file.
 * @param filename the filename
 */
void
ColorTrainTool::load_colormap(const char* filename)
{
  if ( !m_initialized )
    {
      printf("load_colormap: not initialized\n");
      return;
    }

  ColorModelLookupTable* lut = m_generator->get_current();
  lut->load(filename);
}

/** Generate RGB image of the current colormap.
 * @param width width of the generated image
 * @param height height of the generated image
 * @return pointer to the image
 */
unsigned char*
ColorTrainTool::colormap_rgb(unsigned int* width, unsigned int* height)
{
  unsigned int target_width = *width;
  unsigned int target_height = *height;

  unsigned int real_width;
  unsigned int real_height;

  unsigned int lut_width;
  unsigned int lut_height;

  lut_width = lut_height = 512;

  unsigned char* lut_buffer = (unsigned char*) malloc( colorspace_buffer_size(YUV422_PLANAR, lut_width, lut_height) );

  ColorModelLookupTable* lut = m_generator->get_current();
  lut->to_image(lut_buffer);

  float scale_width = target_width / (float) lut_width;
  float scale_height = target_height / (float) lut_height;
  float scale_factor = scale_width < scale_height ? scale_width : scale_height;

  LossyScaler scaler;
  scaler.set_original_buffer(lut_buffer);
  scaler.set_original_dimensions(lut_width, lut_height);
  scaler.set_scale_factor(scale_factor);
  scaler.set_scaled_dimensions(target_width, target_height);
  real_width = scaler.needed_scaled_width();
  real_height = scaler.needed_scaled_height();

  unsigned char* lut_scaled_buffer = (unsigned char*) malloc( colorspace_buffer_size(YUV422_PLANAR, real_width, real_height) );
  
  scaler.set_scaled_buffer(lut_scaled_buffer);
  scaler.scale();
  
  m_lut_rgb_buffer = (unsigned char*) malloc( colorspace_buffer_size(RGB, real_width, real_height) );
  convert(YUV422_PLANAR, RGB, lut_scaled_buffer, m_lut_rgb_buffer, real_width, real_height);

  free(lut_buffer);
  free(lut_scaled_buffer);

  *width = real_width;
  *height = real_height;
  return m_lut_rgb_buffer;
}

/** Sets the source buffer.
 * @param buffer pointer to the image buffer
 * @param width image width
 * @param height image height
 */
void
ColorTrainTool::set_src_buffer(unsigned char* buffer, unsigned int width, unsigned int height)
{
  if ( !m_initialized )
    {
      printf("set_src_buffer: not initialized\n");
      return;
    }

  m_buffer = buffer;
  m_img_width = width;
  m_img_height = height;
  m_img_size = colorspace_buffer_size(YUV422_PLANAR,
				      m_img_width,
				      m_img_height);

  free(m_seg_buffer);
  m_seg_buffer = (unsigned char*) malloc(m_img_size);
  
  m_zauberstab->setBuffer(m_buffer, m_img_width, m_img_height);
}

/** Sets the draw buffer.
 * @param buffer pointer to the draw buffer
 */
void
ColorTrainTool::set_draw_buffer(unsigned char* buffer)
{
  if ( !m_initialized )
    {
      printf("set_draw_buffer: not initialized\n");
      return;
    }
  
  m_draw_buffer = buffer;
}

/** Sets the forground object.
 * @param object the forground object
 */
void
ColorTrainTool::set_fg_object(hint_t object)
{
  if ( !m_initialized ) 
    {
      printf("set_fg_object: not initialized\n");
      return;
    }

  m_fg_object = object;

  delete m_generator;
  m_generator = new BayesColorLutGenerator(m_fg_object);
}

/** Sets the threshold for the magic wand selection.
 * @param threshold the threshold
 */
void
ColorTrainTool::set_threshold(unsigned int threshold)
{
  m_zauberstab->setThreshold(threshold);
}

/** Signal handler to set the magic wand threshold.
 * @param scroll scroll type
 * @param value the threshold volue
 * @return true if signal was handled
 */
bool
ColorTrainTool::set_threshold_sh(Gtk::ScrollType scroll, double value)
{
  set_threshold( (unsigned int) rint(value) );
  
  return true;
}

/** Sets the minimal probability for the colormap generation.
 * @param min_prob the minimal probability
 */
void
ColorTrainTool::set_min_prob(float min_prob)
{
  m_generator->set_min_probability(min_prob);
}

/** Signal handler to set the minimal probability.
 * @param scroll scroll type
 * @param value the minimal probability
 * @return true if signal was handled
 */
bool
ColorTrainTool::set_min_prob_sh(Gtk::ScrollType scroll, double value)
{
  set_min_prob( (unsigned int) rint(value) );
  
  return true;
}

/** Obtain pointer to the buffer containing the segmented image.
 * @return pointer to the segmentation buffer
 */
unsigned char*
ColorTrainTool::seg_buffer()
{
  return m_seg_buffer;
}

/** Check whether color train tool is already initialized.
 * @return true, if initialized
 */
bool
ColorTrainTool::initialized()
{
  return m_initialized;
}

/** Check whether new data that has not been integrated into the
 * current colormap is available.
 * @return true if new data is available
 */
bool
ColorTrainTool::data_available()
{
  return m_data_available;
}
