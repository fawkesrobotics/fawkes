/***************************************************************************
 *  bulb_sampler.h - This header defines a bulb sampler
 *                   bulb sampler is called for given images to take
 *                   measurements eventually resulting in the final LUT
 *
 *  Generated: Tue Jul 26 15:28:51 2005
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
 *             2005       Martin Heracles
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
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __FIREVISION_MODELS_MIRROR_BULB_BULB_SAMPLER_H_
#define __FIREVISION_MODELS_MIRROR_BULB_BULB_SAMPLER_H_

#include "models/scanlines/radial.h"
#include "models/color/lookuptable.h"
#include "models/mirror/bulb.h"
#include "classifiers/simple.h"

class BulbSampler {

 public:

  BulbSampler( unsigned int image_width,
	       unsigned int image_height);

  BulbSampler( unsigned int image_width,
	       unsigned int image_height,
	       unsigned int lut_width,
	       unsigned int lut_height,
	       std::string lut_file);

  BulbSampler( Bulb *bulb,
	       unsigned int image_width,
	       unsigned int image_height,
	       unsigned int lut_width,
	       unsigned int lut_height,
	       std::string lut_file);

  ~BulbSampler();

  void init();
  bool consider(unsigned char *buffer, 
		float x, float y, float ori,
		cart_coord_t *ball_pos_in_image = NULL);
  void consider(unsigned int img_x, unsigned int img_y,
		float x, float y, float ori);

  void finalize();

  void setBallPosition(field_pos_t ballPos);
  void setBallPosition(float ball_x, float ball_y);

  void setCenter(unsigned int imagePointX, 
		 unsigned int imagePointY  );  

  bool calculateOmniOrientation(unsigned char *buffer);
  void calculateOmniOrientation(unsigned int image_x_pos,
				unsigned int image_y_pos );

  float convertAngleW2I(float relativeAngle, float omniOrientation);

  ColorModel *        getColorModel();
  MirrorModel *       getMirrorModel();
  Bulb *              getBulb();

 private:

  unsigned int __image_width;
  unsigned int __image_height;

  Bulb                   *bulb;
  ColorModelLookupTable  *colorLut;
  ScanlineRadial         *radial;
  SimpleColorClassifier *classifier;

  std::list< ROI >           *rois;
  std::list< ROI >::iterator  r;
  
  field_pos_t             ballPosition;

  float getAngle(float referencePointX, float referencePointY,
		 float pointX         , float pointY,
		 float globalAngle                             );

  float getDistance(unsigned int imagePoint1X, unsigned int imagePoint1Y,
		    unsigned int imagePoint2X, unsigned int imagePoint2Y  );

};


#endif
