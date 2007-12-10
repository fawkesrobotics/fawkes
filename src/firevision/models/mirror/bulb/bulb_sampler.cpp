
/***************************************************************************
 *  bulb_sampler.cpp - implementation of bulb sampler
 *
 *  Generated: Tue Jul 26 15:32:16 2005
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
 *             2005       Martin Heracles <Martin.Heracles@rwth-aachen.de>
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

#include <models/mirror/bulb/bulb_sampler.h>
#include <core/exception.h>
#include <iostream>
#include <cmath>

using namespace std;

/** @class BulbSampler <models/mirror/bulb/bulb_sampler.h>
 * Bulb sampler.
 * The bulb sampler takes samples from images taken by the camera and annotated
 * with information about the ball position (either automatically or manually).
 * @author Martin Herakles
 * @author Tim Niemueller
 */

/** Constructor.
 * @param image_width width of camera images
 * @param image_height height of camera images
 * @param color_lut_width width of color LUT
 * @param color_lut_height height of color LUT
 * @param lut_file COlor lut file that is loaded
 */
BulbSampler::BulbSampler( unsigned int image_width,
			  unsigned int image_height,
			  unsigned int color_lut_width,
			  unsigned int color_lut_height,
			  string lut_file)
{
  __image_width  = image_width;
  __image_height = image_height;

  bulb    = new Bulb( __image_width, __image_height );

  try {
    colorLut   = new ColorModelLookupTable ( (char *)lut_file.c_str(), 
  					   color_lut_width, color_lut_height );
  } catch (Exception &e) {
    throw;
  }

  radial     = new ScanlineRadial( __image_width    , __image_height, 
				   __image_width / 2, __image_height / 2,
				   5, 5, 10);

  classifier = new ReallySimpleClassifier( radial, 
					   colorLut,
					   0                          ); // min_num_points

}


/** Constructor.
 * This constructor allows for using the bulb sampler without a color model,
 * this means without auto-ball detection. Note that you will not be able
 * to call the consider() method that takes an image buffer as parameter.
 * @param image_width width of camera images
 * @param image_height height of camera images
 */
BulbSampler::BulbSampler( unsigned int image_width,
			  unsigned int image_height )
{
  __image_width = image_width;
  __image_height = image_height;
  bulb    = new Bulb( image_width, image_height );

  colorLut = NULL;
  radial = NULL;
  classifier = NULL;
}


/** Constructor.
 * This uses the supplied bulb mirror model.
 * @param bulb bulb mirror model
 * @param image_width width of camera images
 * @param image_height height of camera images
 * @param color_lut_width width of color LUT
 * @param color_lut_height height of color LUT
 * @param lut_file colormap file name
 */
BulbSampler::BulbSampler( Bulb *bulb,
			  unsigned int image_width,
			  unsigned int image_height,
			  unsigned int color_lut_width,
			  unsigned int color_lut_height,
			  string lut_file)
{
  __image_width = image_width;
  __image_height = image_height;
  this->bulb = bulb;

  colorLut   = new ColorModelLookupTable ( (char *)lut_file.c_str(), 
					   color_lut_width, color_lut_height );

  radial     = new ScanlineRadial( image_width  , image_height, 
				   image_width/2, image_height/2,
				   5, 5,
				   10                              );

  classifier = new ReallySimpleClassifier( radial,  colorLut,
					   /* min num points */ 0);

}


/** Destructor. */
BulbSampler::~BulbSampler() {
  delete classifier;
  delete radial;
  delete colorLut;
  delete bulb;
}


/** Empty init. */
void
BulbSampler::init()
{  
}


/** Take measurement
 * Use classifier to detect ball.
 * @param buffer buffer in YUV422_PLANAR with current image
 * @param x X-Position of robot on field in world coordinates
 * @param y Y-Position of robot on field in world coordinates
 * @param ori Orientation
 * @param ball_pos_in_image upon return contains the ball position in the image
 * @return true if image was successfully considered, false otherwise
 */
bool
BulbSampler::consider(unsigned char *buffer,
		      float x, float y, float ori,
		      cart_coord_t *ball_pos_in_image) {

  if ( classifier == NULL ) {
    throw Exception("Cannot use this consider method in non-detection mode!");
  }

  // find the ball in the image
  cart_coord_t ballPosInImage;
  ballPosInImage.x = 0;
  ballPosInImage.y = 0;
  classifier->set_src_buffer( buffer, __image_width, __image_height );
  rois = classifier->classify();
  if ( rois->empty() ) {
    cout << "No ROIs!" << endl;
    delete rois;
    return false;
  }

  // there are ROIs, now find the ROI that contains the ball

  // direction in which the robot has to search for the ball
  // relative to robot:
  float ballDirectionRelative;
  ballDirectionRelative = getAngle( x, y, 
				    ballPosition.x, ballPosition.y,
				    ori                             );     
  // absolute (in omni image):
  float ballDirectionInImage;
  ballDirectionInImage = convertAngleW2I( ballDirectionRelative, bulb->getOrientation() );

  // find the ROI whose mass point direction deviates least from "ballDirectionInImage"
  float deviation = 0.0;
  float minDeviation = 2 * M_PI;
  cart_coord_t massPoint;
  bool ballFound = false;
  for (r = rois->begin(); r != rois->end(); ++r) {
    if (r->hint == H_BALL) {
      // consider mass point of ball
      classifier->getMassPointOfBall( &(*r), &massPoint );
	
      /*
      if (display != NULL) {
	display->drawCircle( massPoint.x, massPoint.y, 5 );
	display->drawCircle( massPoint.x, massPoint.y, 4 );
	display->drawCircle( massPoint.x, massPoint.y, 3 );
	display->drawCircle( massPoint.x, massPoint.y, 2 );
	display->drawCircle( massPoint.x, massPoint.y, 1 );
      }
      */

      deviation = getAngle( (bulb->getCenter()).x, (bulb->getCenter()).y,
			    massPoint.x             , massPoint.y             ,
			    ballDirectionInImage                                );
      if ( fabs(deviation) < minDeviation ) {
	ballFound = true;
	ballPosInImage.x = massPoint.x;
	ballPosInImage.y = massPoint.y;
	minDeviation = deviation;
      }
    } 
  } 

  rois->clear();
  delete rois;
    
  if ( ballFound ) {
    
    cout << "(BulbSampler::consider): " << endl
	 << "  Searching in direction: " << ballDirectionInImage << endl
	 << "    Relative direction  : " << ballDirectionRelative << endl;

    consider(ballPosInImage.x, ballPosInImage.y, x, y, ori);

    if (ball_pos_in_image != NULL) {
      ball_pos_in_image->x  = ballPosInImage.x;
      ball_pos_in_image->y  = ballPosInImage.y;
    }
  }


  return ballFound;
}


/** Take measurement
 * Use the given image position and store a mapping to the given world
 * coordinates. This can be used for manual mode.
 * @param img_x image x position of the ball
 * @param img_y image y position of the ball
 * @param x X-Position of robot on field in world coordinates
 * @param y Y-Position of robot on field in world coordinates
 * @param ori Orientation
 */
void
BulbSampler::consider(unsigned int img_x, unsigned int img_y,
		      float x, float y, float ori)
{
  // update lookuptable
  float distX = x - ballPosition.x;
  float distY = y - ballPosition.y;
  // for simplicity, we use the angle to the ball in the image (not in world)
  float angle = getAngle( (bulb->getCenter()).x, (bulb->getCenter()).y,
			  img_x        , img_y        ,
			  0.0                                                 );
  bulb->setWorldPoint( img_x, img_y, 
		       sqrt( distX*distX + distY*distY ), angle);


  cout << "(BulbSampler::consider): " << endl
       << "  Ball position in image: (" << img_x << ", " << img_y << ")" << endl
       << "    ...has angle        : " << angle << endl
       << "  Ball in World" << endl
       << "    Distance to robot   : " << sqrt( distX*distX + distY*distY ) << endl;

}


/** Finalize. */
void
BulbSampler::finalize()
{
}


/** Set ball position.
 *  Ball position is the (global) real-world position
 * where the ball has been placed.
 * @param ballPos ball position
 */
void
BulbSampler::setBallPosition(field_pos_t ballPos) {
  this->ballPosition.x = ballPos.x;
  this->ballPosition.y = ballPos.y;
  cout << "(BulbSampler::setBallPosition): Ball position set to ("
       << this->ballPosition.x << ", " << this->ballPosition.y << ")." << endl;
}


/** Set ball position.
 * Ball position is the (global) real-world position
 * where the ball has been placed.
 * @param ball_x x coordinate of the ball position
 * @param ball_y y coordinate of the ball position
 */
void
BulbSampler::setBallPosition(float ball_x, float ball_y) {
  this->ballPosition.x = ball_x;
  this->ballPosition.y = ball_y;
  cout << "(BulbSampler::setBallPosition): Ball position set to ("
       << this->ballPosition.x << ", " << this->ballPosition.y << ")." << endl;
}


/** Set center.
 * Defines the image center of the bulb mirror model.
 * @param imagePointX center x coordinate in image
 * @param imagePointY center y coordinate in image
 */
void
BulbSampler::setCenter(unsigned int imagePointX,
		       unsigned int imagePointY  ) {
  this->bulb->setCenter( imagePointX, imagePointY );
}


/** Get angle.
 *  Returns the (smallest) angle between 
 *  the ray from (referencePointX, referencePointY) thru (pointX, pointY), and
 *  the ray from (referencePointX, referencePointY) in direction "globalAngle".
 * @param referencePointX reference point X
 * @param referencePointY reference point Y
 * @param pointX point Y
 * @param pointY point Y
 * @param globalAngle global angle
 */
float
BulbSampler::getAngle( float referencePointX, float referencePointY,
		       float pointX         , float pointY,          
		       float globalAngle                             )
{
  float angle = 0.0;

  /* calculate the absolute angle to (pointX, pointY),
     which is the angle between direction "to (pointX, pointY)" and direction "to the right of the soccer field"
     (with respect to reference point) */
  if (pointX >= referencePointX) {
    angle = atan( float(pointY - referencePointY) / 
		  float(pointX - referencePointX)   );
  } 
  else if (pointY >= referencePointY) {
    angle = M_PI + atan( float(pointY - referencePointY) / 
			 float(pointX - referencePointX)   );
  } 
  else {
    angle = -M_PI + atan( float(pointY - referencePointY) / 
			  float(pointX - referencePointX)   );
  }

  /* calculate the relative angle to (pointX, pointY),
     by taking "globalAngle" into consideration */
  if ( (angle    >= 0.0 && 
	globalAngle >= 0.0    ) || 
       (angle    <  0.0 && 
	globalAngle <  0.0    )    ) {
    return angle - globalAngle;
  } 
  else {
    /* "angle1" is the angle between "globalAngle" and "angle" that 
       does contain the direction "to the right of the soccer field" */
    float angle1 = fabs(globalAngle) + fabs(angle);
    // "angle2" is the angle complementary to "angle1"
    float angle2 = 2.0 * M_PI - angle1;

    if (angle1 < angle2) {	
      // angle1 is correct, but the sign has to be determined
      if (globalAngle < 0.0) {
	return angle1;
      } 
      else {
	return -angle1;
      }
    } 
    else {			
      // angle2 is correct, but the sign has to be determined
      if (globalAngle >= 0.0) {
	return angle2;
      } 
      else {
	return -angle2;
      }
    }
  }
}


/** Get distance.
 * Returns the (euklidian) distance between two image points
 * @param imagePoint1X point 1 x
 * @param imagePoint1X point 1 y
 * @param imagePoint1X point 2 x
 * @param imagePoint1X point 2 y
 * @return distance from point 1 to point 2
 */
float 
BulbSampler::getDistance(unsigned int imagePoint1X, unsigned int imagePoint1Y,
			 unsigned int imagePoint2X, unsigned int imagePoint2Y  ) {
  float diffX = float(imagePoint1X) - float(imagePoint2X);
  float diffY = float(imagePoint1Y) - float(imagePoint2Y);
  return sqrt( diffX * diffX +
	       diffY * diffY   );
}


/** Calculate omni orientation.
 * Automatic mode, tries to find a ball. If found assumes that it is lying in front
 * of the robot and from this derives the fron position of the ball.
 *   If the robot is facing the ball, 
 *   which is the ball-direction in the omni-image?
 *   This angle is called the orientation of the omni-camera.
 * @param buffer image buffer.
 * @return true on success, false otherwise.
 */
bool
BulbSampler::calculateOmniOrientation(unsigned char *buffer) {
  // find the ball in the image
  cart_coord_t ballPosInImage;
  ballPosInImage.x = 0;
  ballPosInImage.y = 0;
  classifier->set_src_buffer( buffer, __image_width, __image_height );
  rois = classifier->classify();
  if ( rois->empty() ) {
    cout << "(BulbSampler::calculateOmniOrientation): No ROIs!" << endl;
    delete rois;
    return false;
  }

  // there are ROIs, now find the closest (ball-)ROI

  float distance = 0.0;
  float minDistance = 999999.0;
  cart_coord_t massPoint;
  bool ballFound = false;
  // for each ball-ROI
  for (r = rois->begin(); r != rois->end(); ++r) {
    if (r->hint == H_BALL) {
      // consider distance from omni-camera center to mass point of ball
      classifier->getMassPointOfBall( &(*r), &massPoint );
      distance = getDistance( (bulb->getCenter()).x, (bulb->getCenter()).y,
			      massPoint.x             , massPoint.y               );
      if (distance < minDistance) {
	ballFound = true;
	ballPosInImage.x = massPoint.x;
	ballPosInImage.y = massPoint.y;
	minDistance = distance;
      }
    } 
  } 

  rois->clear();
  delete rois;
    
  if ( ballFound ) {      
    calculateOmniOrientation( ballPosInImage.x, ballPosInImage.y );
  } 

  return ballFound;
}


/** Calculate omni orientation.
 * Manual mode version. Assume the given point (x,y) to be the front.
 * @param x image point x
 * @param y image point y
 */
void
BulbSampler::calculateOmniOrientation(unsigned int x, unsigned int y)
{

  // calculate orientation of omni-camera
  float orientation;
  orientation = getAngle( (bulb->getCenter()).x, (bulb->getCenter()).y,
			  x, y, 0.0 );
  bulb->setOrientation( orientation ); 
  cout << "BulbSampler: Orientation of omni cam: " << orientation << " rad" << endl;
  
}



/** Convert angle.
 *  If, relative to the robot, the ball is at direction "relativeAngle",
 *  which is the ball-direction in the omni-image?
 * @param relativeAngle relative angle
 * @param omniOrientation omni orientation
 * @return angle
 */
float 
BulbSampler::convertAngleW2I(float relativeAngle, float omniOrientation) {
  float tmpAngle = 0.0;

  // convert without taking into consideration orientation of omni-camera
  if (relativeAngle >= 0.0 && 
      relativeAngle <= M_PI   ) {
    // positive 
    tmpAngle = -relativeAngle + M_PI;
  } 
  else if (relativeAngle <= 0.0 && 
	   relativeAngle >= -M_PI  ) {
    // negative
    tmpAngle = -relativeAngle - M_PI;
  } 
  else {
    // invalid angle
    cout << "(BulbSampler::convertAngleW2I): Error!" << endl
	 << "                                Angle must be in interval [-PI, PI]." << endl;
    return 0.0;
  }

  // turn around by PI, because currently the angle points to the opposite direction
  if (tmpAngle + M_PI >= -M_PI &&
      tmpAngle + M_PI <= M_PI     ) {
    tmpAngle += M_PI;
  }
  else if (tmpAngle + M_PI > M_PI) { 
    tmpAngle = -( M_PI - tmpAngle );
  }
  else {
    // "tmpAngle + M_PI < -M_PI"
    tmpAngle = M_PI - ( -(tmpAngle + M_PI) - M_PI );
  }

  
  // shift angle by orientation of omni-camera
  if (-M_PI <= tmpAngle + omniOrientation && tmpAngle + omniOrientation <= M_PI)
    {
      tmpAngle = tmpAngle + omniOrientation;
    }
  else if (tmpAngle + omniOrientation > M_PI)
    { 
      tmpAngle = -1.0 * ( M_PI - ((tmpAngle + omniOrientation) - M_PI) );

    }
  else // "tmpAngle + omniOrientation < -M_PI"
    {
      tmpAngle = M_PI - ( (-1.0 * (tmpAngle + omniOrientation)) - M_PI );
    }

  cout << "Searching ball in direction " << tmpAngle << " in omni-image." << endl << endl;
  

  return tmpAngle;
}


/** Get color model.
 * @return used color model.
 */
ColorModel *
BulbSampler::getColorModel()
{
  return colorLut;
}


/** Get mirror model.
 * @return mirror model
 */
MirrorModel *
BulbSampler::getMirrorModel()
{
  return bulb;
}


/** Bulb mirror model.
 * @return bulb mirror model.
 */
Bulb *
BulbSampler::getBulb()
{
  return bulb;
}
