#include "UserDef.hh"

#ifndef __aux__hh__
#define __aux__hh__

/***************************************************************************
 *  Auxillary.hh - Header file for object recognition with random forests: Auxillary Components
 *
 *  Created: April 18/2008
 *  Copyright  2008  Vaishak Belle
 *
 * 
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

/* 
   ----------- Auxillary Re-usable functions -------------
*/

using namespace std;


namespace Auxillary {


  /**
   * class to generate random numbers form boost libraries 
   */
  class RandomGeneratorClass{ 

  private:
    static RandomGeneratorClass* instance;

    /** empty constructor to model as singleton object */ 
    RandomGeneratorClass(){
    }

        
  public:    
    /** singleton object */ 
    static RandomGeneratorClass* getRandomGeneratorInstance(){ 
      if(instance==NULL)
	instance = new RandomGeneratorClass();
      
      return instance;
    }
    
    /** generate random threshold */ 
   double randomThreshold(){
      RandomGeneratorClass* inst;
      inst = getRandomGeneratorInstance();
      //The class is not correctly used!!!! 
      return 0.0;
   }
    
  };

  
  /** Checks if X in the range of (Y-range, Y+range) */ 
  bool inRange(int X, int Y, int range);
    
  
  /**  A simple check and swap function to make sure theta1 < f < theta2  */ 
  void checkAndSwap(int *a, int *b);
  
  /**   Find the minimum among the double values - (entropies)  */ 
  int findMin(double *, int); 
  
  /** Random number generator */
  int randomNumber();
  
  /** x_ & y_ : Depenedent on RECTANGULAR_FEATURE  */
  int randomRectangle();
  
  /** Feature pass */
  bool featurePass(double,double,double, int nosThresholds); 
  
  class UserDef::ConfigClass; 


  /** Random number for x - row */ 
  int randRowIndex(UserDef::ConfigClass& );

  /** Random number for col */
  int randColIndex(UserDef::ConfigClass& );


  

  /** Integer to String */ 
  string itos(int);

  class UserDef::VectorOfIntegralImages; 

  /** Test module to investiage integral images */
  void writeOutIntegralImages(UserDef::VectorOfIntegralImages* integralImages, 
			      int classNumber, int height, int width, 
			      std::ostream& out);

 
  /** Write down detection postitives */
  int writeToFile(std::vector<UserDef::WindowClass*>, std::ofstream& );

  

  //These writeToFile and readFromFile occur in pairs - name overloading - for specific datastructures. 

  /** Write to file (for integral images) */ 
  int writeToFile(UserDef::VectorOfIntegralImages*, string writeLocation, int height, int width);
  /** read from file ( for integral images) */ 
  int readFromFile(UserDef::VectorOfIntegralImages* integralImages, string readLocation, int height, int width);

  /** Writing the config instance to a file */
  int writeToFile(UserDef::ConfigClass &);
  /** read the config instance */ 
  int readFromFile(UserDef::ConfigClass& );

  /** print config instance */ 
  int printConfigWindows( UserDef::ConfigClass& config, vector< UserDef::WindowClass* > &windows); 



//   //Writing an instance of Test Class (- the integral images are not written!)
//   int writeToFile(std::ostream& out, Forest::Test& test);
//   int readFromFile(std::istream& in, Forest::Test& test); 

//   //Write a tree to a file (-only the structure and tests are written!)
//   int writeToFile(std::ostream& out, std::ostream& testOut, Forest::Tree&);
//   int readFromFile(std::istream& in, std::istream& testIn, Forest::Tree**);
}

#endif
