
/***************************************************************************
 *  forest_aux.h auxillary functions for the random forest implementation
 *
 *  Created: 07 05 2008 
 *  Copyright  2008 Vaishak Belle
 *
 *  $Id$ 
 *
 ****************************************************************************/


/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#include <vector> 
#include <opencv/cv.h> 


#ifndef __FIREVISION_FVUTILS_RECOGNITION_FOREST_FOREST_AUX_H_
#define __FIREVISION_FVUTILS_RECOGNITION_FOREST_FOREST_AUX_H_


/**
 * Template class for IplImage Handlers
 */
template<class t> class Image { 
 private:
  /** the iplImage */ 
  IplImage *imgp;
     
 public:
  /** constructor 
   * \param img the IplImage passed */ 
  Image(IplImage* img=0){ imgp = img; }
  /** destructor */ 
  ~Image(){imgp = 0;}
  /** operator  = overloaded 
   * \param img the IplImage */ 
  void operator=(IplImage* img){ imgp = img;}
  /** operation [] is overloaded for easy array lookup
   * \param rowIndx the row index for array lookup 
   */ 
  inline t* operator[](const int rowIndx){
    return ((t *)(imgp->imageData + rowIndx*imgp->widthStep));}
  /** simple return of the IplImage stored */ 
  inline IplImage* getIplImage(){ return imgp;} 
};
  
/** 
 * Quick access of RGB pixel intensities
 */
typedef struct{ 
  /** b blue pixel */
  unsigned char b; 
  /** g green pixel */    
  unsigned char g;
  /** r red pixel */ 
  unsigned char r;
} RgbPixel;
  

typedef Image<RgbPixel> RgbImage;



/**
 * Vector of integral images - stores the individual integral images of each imag We consider an array of VectorOfIntegralImages for the training collections for all classes. 
 */
class VectorOfIntegralImages {
 private:
  /** height of the training images */ 
  int heightOfEachImage;
  /** width of the trainig images */ 
  int widthOfEachImage;
    
 public:
  /** Storing the pointers to the integral-image array */
  std::vector< int* > iiVector; 
    
  /** returns the size of the vector */ 
  int size() const { return iiVector.size(); } 
  /** clears the items of the vector */ 
  void clear() { iiVector.clear(); } 

  /** sets the height of each image in the vector - the images are assumed to be scaled to the same size 
   * @param height set the height to this integer 
   */ 
  void setHeight(int height) { heightOfEachImage = height; }
  /** set the width of each image in the vector  - the images are assumed to be scaled to the same size 
   * @param width set the width to this integer value */ 
  void setWidth(int width) { widthOfEachImage = width; }
  /** returns the height of an image - the images are assumed to be scaled to the same size */
  int getHeight() { return heightOfEachImage; }
  /** returns the width of an image - the images are assumed to be scaled to the same size */ 
  int getWidth() { return widthOfEachImage; }
};

/** get the rectangular feature score */ 
double getRectangularIntegralImagefeature(int* integralImageCalculated, int xMain, int yMain, int x, int y, int height, int width, int haarFeature);

/** x_ & y_ : Depenedent on RECTANGULAR_FEATURE  */
int randomRectangle();

/** Feature pass */
bool featurePass(double,double,double, int nosThresholds); 

/**   Find the minimum among the double values - (entropies)  */ 
int findMin(double *, int); 



/** calculating the integral image */
void calculateIntegralImage(IplImage* image, int *integralImage); 




#endif
