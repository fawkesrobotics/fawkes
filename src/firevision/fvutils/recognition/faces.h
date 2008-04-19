
/***************************************************************************
 *  faces.h - Face Recognizer
 *
 *  Created: Wed Dec 12 13:04:12 2007
 *  Copyright  2007  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_FVUTILS_RECOGNITION_FACES_H_
#define __FIREVISION_FVUTILS_RECOGNITION_FACES_H

#include <fvutils/base/roi.h>
#include <vector> 
#include <map>
#include <list>
#include <string>
#include <vector>

typedef struct _IplImage IplImage;

class FaceRecognizer
{
 private:
  /** location of the training images */
  char __training_images_location[PATH_MAX]; 
  /** number of identities */ 
  int __n_identities;
  /** forest size */
  int __forest_size;
  /** map of identity indices to person names */
  std::map<int, std::string> __person_names; 

 public:
  FaceRecognizer(const char* loc, int number_of_identities, int forest_size );
  ~FaceRecognizer();

  /** a vector containing the identities of the faces supplied */ 
  typedef std::vector<int> Identities; 

  
  void add_identity( int, std::string );

  //  Identities recognize(unsigned char *buffer, std::list<ROI> &rois, int number_of_identities );
  Identities   recognize(std::vector<IplImage *> faces, int number_of_identities );
  std::vector<std::string>  get_identities( Identities& ); 



};


#endif
