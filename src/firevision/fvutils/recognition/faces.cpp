
/***************************************************************************
 *  faces.cpp - Face Recognizer
 *
 *  Created: Wed Dec 12 13:08:42 2007
 *  Copyright  2007  Tim Niemueller [www.niemueller.de]
 *             2008  Vaishak Belle
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

#include <fvutils/recognition/faces.h>
#include <fvutils/recognition/forest/forest.h>
#include <fvutils/recognition/forest/forest_aux.h>
#include <fvutils/recognition/forest/forest_param_default.h>



/** @class FaceRecognizer <fvutils/recognition/faces.h>
 * Face Recognizer.
 * This class takes an image and a number of ROIs as input. 
 * the set of recognized identities are returned 
 * @author Tim Niemueller, Vaishak Belle
 */


/** constructor that takes in the training images location 
 * @param loc the location of the training images on disk 
 * @param number_of_identities is the number of identities present in the training location
 * @param forest_size the size of the random forest we want to grow 
 */
FaceRecognizer::FaceRecognizer(const char* loc, int number_of_identities, int forest_size )
{
  strcpy( __training_images_location, loc ); 
  __n_identities = number_of_identities;
  __forest_size = forest_size;
  __config = new ForestConfigClass( __n_identities ); 
  //  __config = new UserDef::ConfigClass( __n_identities );
  __train_height = __train_width = 0; 
  //    __forest = new Forest::ForestClass( __training_images_location, __n_identities, __train_height, __train_width, 
  //			      *__config, __forest_size );
    __forest = new ForestClass( __training_images_location, __n_identities, __train_height, __train_width, *__config, __forest_size ); 
  

 }


/** Generic Destructor. */
FaceRecognizer::~FaceRecognizer()
{
  delete __forest; 
  delete __config; 
}


/** Recognize faces.
 * Scans the given ROIs in the image for faces and returns a map of labeled
 * faces.
 * @param faces IplImages of faces
 * @param number_of_identities it specifies the number
 * @return the list of corresponding recognized identities 
 */
FaceRecognizer::Identities 
FaceRecognizer::recognize(vector<IplImage*> faces, int number_of_identities )
{

  if( number_of_identities != 0 )
    {
      // @TODO introduce rebuilding the forest 
    }

  FaceRecognizer::Identities identities;
  
//   int rescaling_height = 0, rescaling_width = 0;   // the detected faces need to be scaled to the following size 
//   UserDef::ConfigClass config( this->__n_identities );   // creating an instance of the config class

  //BUILD FOREST
//   Forest::ForestClass *theForest = new Forest::ForestClass( this->__training_images_location, this->__n_identities, rescaling_height, 
// 							    rescaling_width, config,  this->__forest_size);
  CvSize rescaling_size = cvSize( getTrainWidth(), getTrainHeight() ); 

  
  IplImage* face; 
  for( unsigned int i = 0; i < faces.size(); i++ ) 
    { 
      face = cvCreateImage( rescaling_size, faces.at(i)->depth, faces.at(i)->nChannels ); 
      cvResize( faces.at(i), face, CV_INTER_LINEAR );
      identities.push_back( getClassLabelFromForest( __forest, face ) ); 
      //      identities.push_back( Forest::getClassLabelFromForest( __forest, face ) ); 
      cvReleaseImage( &face ); 
    }

  //delete theForest;
  return identities; 
}


/** Add identity to internal conversion map.
 * @param person_index person of index
 * @param person_name name/label for this person
 */
void 
FaceRecognizer::add_identity( int person_index , std::string person_name )
{ 
  //__n_identities++; 
  __person_names[person_index] = person_name; 
}

/** Transform person indexes to labels.
 * @param identities vector of identities to convert
 * @return vector of labels, indexes of vector correspond to the indexes in the given
 * identities vector.
 */
std::vector<std::string> 
FaceRecognizer::get_identities( Identities identities )
{ 
  std::vector<std::string> person_names;
 
  for( Identities::iterator i = identities.begin(); i != identities.end(); ++i ) {
    printf("Processing Person %d, name is %s\n", *i, __person_names[*i].c_str());
    person_names.push_back( __person_names[*i] );
  }
  
  return person_names; 
}
  

