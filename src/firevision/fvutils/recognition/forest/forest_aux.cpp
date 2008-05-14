/***************************************************************************
 *  forest_aux.cpp auxillary functions for the random forest implementation
 *
 *  Created: Wed Dec 12 13:04:12 2008
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


#include <utils/logging/liblogger.h>
#include "forest_param_default.h" 
#include <iostream>
#include <math.h> 
#include <opencv/cv.h> 
#include "forest_aux.h" 



//Haar Filter Set used
enum {BLOCK , RECT2_HORIZONTAL , RECT2_VERTICAL, RECT3_VERTICAL};




/** get the rectangular feature score */ 
double get_rectangular_integral_image_feature(int* integralImageCalculated, int xMain, int yMain, int x, int y, int height, int width, int haarFeature){
    
  double normalizedII;
  
  int sum =0;
  int sum1, sum2, sum3;
  int breadth;
    
  int i=0, j=0, ii=0, jj=0;
  int tl, tr, bl, br; // coordinates of the rectangle - topleft, topright, bottomleft, bottomright
    
  int temp1 =0, temp2 =0, temp3, temp4;
    
  /*the bounds are from 0 to width-2*/
  if(xMain>(height-2)) xMain = height-2;
  if(yMain>(width-2)) yMain =width-2;
  if(x>(height-2)) x = height-2;
  if(y>(width-2)) y = width-2;
  
  /*first let us establish the bigger and samller */
  if(xMain>x){
    
    i = x;
    ii= xMain;
  }
  else{
      
    i = xMain;
    ii= x;
  }

  if(yMain>y){
      
    j = y;
    jj = yMain;
  }
  else {
      
    j = yMain;
    jj = y;
  }
    
  /*now if the combination formes a rectangle within bounds then, we can consider (i,j) to be startin coordinates and (ii,jj) to be breadth and legnth */
  /* so, the coordinates are in tl, tr, bl, br order: (i,j), (i, j+jj), (i+ii, j), (i+ii, j+jj) */
  if((i+ii)<(height-1) && (j+jj)<(width-1)){
    
    tl = i*width + j;
    tr = i*width + (j+jj);
    bl = (i+ii)*width + j;
    br = (i+ii)*width + (j+jj);

    height = ii;
    breadth = jj;
  }

  /*now, the corrdinates are (i,j), (i, jj), (i+ii, j), (i+ii, jj) */
  if((i+ii)<(height-1) && (j+jj)>(width-1)){
      
    tl = i*width + j;
    tr = i*width + jj;
    bl = (i+ii)*width +j;
    br = (i+ii)*width + jj;

    height = ii;
    breadth = (jj-j);
  }
  /*now the coordinates are (i,j), (i, j+jj), (ii, j) , (ii, j+jj) */
  if((i+ii)>(height-1) && (j+jj)>(width-1)){

    tl = i*width + j;
    tr = i*width + (j+jj);
    bl = ii*width + j;
    br = ii*width + (jj+j);

    height = (ii-i);
    breadth = jj;
  }

  /* now the corrdinates are (i,j), (i, jj), (ii, j) and (ii,jj) */
  else{

    tl = i*width +j;
    tr = i*width + jj;
    bl = ii*width + j;
    br = ii*width + jj;

    height = (ii-i);
    breadth = (jj-j);
  }

  if(haarFeature == BLOCK)
    sum = integralImageCalculated[tl] + integralImageCalculated[br] - 
      integralImageCalculated[tr] - integralImageCalculated[bl];

  if(haarFeature == RECT2_HORIZONTAL){

    temp1 = ((int)floor(ii/2))*width + j;
    temp2 = ((int)floor(ii/2))*width + jj;

    sum1 = integralImageCalculated[tl] + integralImageCalculated[temp2] 
      - integralImageCalculated[temp1] - integralImageCalculated[tr];

    sum2 = integralImageCalculated[temp1] + integralImageCalculated[br] 
      - integralImageCalculated[bl] - integralImageCalculated[temp2];

    sum = sum1 - sum2;
    height /=2;
    //   breadth /=2; //iforgot that its not h/2 and b/2 but only one needs to be divided
  }

  if(haarFeature == RECT2_VERTICAL){
    
    temp1 = i*width + ((int)floor(jj/2));
    temp2 = ii*width + ((int)floor(jj/2));

    sum1 = integralImageCalculated[tl] + integralImageCalculated[temp2] - integralImageCalculated[bl] - integralImageCalculated[temp1];
      
    sum2 = integralImageCalculated[temp1] + integralImageCalculated[br] - integralImageCalculated[temp2] - integralImageCalculated[tr];
      
    sum = sum1 - sum2;
    //  height /=2;
    breadth /=2;
  }


  if(haarFeature == RECT3_VERTICAL){

    temp1 = i*width + ((int)floor(jj/3));
    temp2 = ii*width + ((int)floor(jj/3));
    temp3 = i*width + ((int)floor(2*jj/3));
    temp4 = ii*width + ((int)floor(2*jj/3));

    sum1 = integralImageCalculated[tl] + integralImageCalculated[temp2] - integralImageCalculated[bl] - integralImageCalculated[temp1];

    sum2 = integralImageCalculated[temp1] + integralImageCalculated[temp4] - integralImageCalculated[temp2] -integralImageCalculated[temp3];

    sum3 = integralImageCalculated[temp3] + integralImageCalculated[br] - integralImageCalculated[tr] - integralImageCalculated[temp4];

    sum = sum1 + sum3 - sum2;
    // height /=3;
    breadth /=3;
  }
  
  if(height<0 || height==0) height =1;
  if(breadth<0 || breadth==0) breadth =1;

  normalizedII = (double)sum/(255*height*breadth + 1.0);

  return normalizedII;
}

/** return a random rectangle dimension 
 * returns a random dimension 
 */ 
int random_rectangle() 
{
  return rand()%RECTANGULAR_FEATURE_SIZE; 
}

/** feature pass */
bool feature_pass(double feature, double theta1, double theta2, int number){

  if(number == 2) { 

    if((feature>theta1||feature==theta1) && (feature<theta2||feature==theta2))
      return true;

    else 
      return false;
  }

  else { 

    if(feature>theta1 || feature==theta1)
      return true;

    else
      return false; 
  } 
}




/** find the minimum among the double values - (entropies) */
int find_min( double* entropies=0, int size=0){

  int finalIndex = -1;
  int i =0;

  double finalEntropy = 2.0;
    
  if(size<0){
    LibLogger::log_error("Auxillary.cpp","entropy array is empty"); 
    return 0;
  }

  finalEntropy = entropies[0];
  finalIndex = 0;

  for(i=1;i<size;i++){

    if(entropies[i]<finalEntropy) {
	
      finalEntropy = entropies[i];
      finalIndex = i;
    }
  }

  return finalIndex;
}



/** For a given image at a location, calculate its integralimage */ 
void calculate_integral_image(IplImage* image=0, int *integralImage=0)
{

  const int height = image->height;
  const int width = image->width;

  int ii[height][width];
  int s[height][width];



  RgbImage img( image );
  int i=0,j=0;
    
  for(i=0;i<height-1;i++)
    for(j=0;j<width-1;j++){

      if(j==0)
	s[i][j] = 0 + img[i][j].r;

      else
	s[i][j] = s[i][j-1] + img[i][j].r;
	
      if(i==0)
	ii[i][j] = 0 + s[i][j];

      else
	ii[i][j] = ii[i-1][j] + s[i][j];
    }
    
  for(i=0;i<height-1;i++)
    for(j=0;j<width-1;j++)
      if(ii[i][j]<0) { 
	LibLogger::log_error("UserDef.cpp","the integral image value is lesser than 0"); 
      }
      else
	integralImage[i*width + j] = ii[i][j]; 
    
}
