#include "UserDef.hh"
#include <utils/logging/liblogger.h>

/***************************************************************************
 *  Auxillary.cpp - Src file for object recognition with random forests: Auxillary Components 
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

boost::minstd_rand g;
boost::uniform_01<boost::minstd_rand> randBoost(g);

/** rand boost */ 
double randomBooster(){
  return randBoost();
}


namespace Auxillary{ 

  
  
  /** range checker */
  bool inRange(int x, int a, int range){ 
    int greater, lesser; 

    if( x<a ) { 

      greater = a ;
      lesser = x ;
    } 
    
    else if( a<x ) { 

      greater = x;
      lesser = a; 
    } 

    else // they are equal, so in range
      return true;

    if( greater < (lesser + range) || greater == (lesser + range) )
      return true;

    else 
      return false; 
  }
    
    


  /** A simple check and swap function to make sure theta1 < f < theta2 */
  void checkAndSwap(int *a=0, int *b=0) { 

    int temp;

    if(*a > *b) { /* first threshold is greater than second */ 

      temp = *a;
      *a = *b;
      *b = temp;
    }
  }

  /** find the minimum among the double values - (entropies) */
  int findMin( double* entropies=0, int size=0){

    int finalIndex = -1;
    int i =0;

    double finalEntropy = 2.0;
    
    if(size<0){

      //      perror("The entropy array is empty - it needs to contain values");
      LibLogger::log_error("Auxillary.cpp","entropy array is empty"); 
      //      exit(0);
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


  /** a random number generator */
  int randomNumber(){

    return rand();
  }
  
  /** To generate x_' and y_' */
  int randomRectangle(){

    return randomNumber()%RECTANGULAR_FEATURE_SIZE; 
  }

  /** feature pass */
  bool featurePass(double feature, double theta1, double theta2, int number){

    //    return ((feature>theta1||feature==theta1) && (feature<theta2||feature==theta2));

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

  /** random number for x - row  */ 
  int randRowIndex(UserDef::ConfigClass& config){

    return randomNumber()%config.imageInfoInstance.getHeight();
  }

  /** random number for col */ 
  int randColIndex(UserDef::ConfigClass& config){

    return randomNumber()%config.imageInfoInstance.getWidth();
  }
  
  /** Integer to String */ 
  string itos(int i){

    char *buffer = new char[30];

    // std::itoa not found!!!
    //  std::itoa(i, buffer, 10); 
    
    return buffer;
  }


  /** PRINT OUT TO CONSOLE ( NOT TO WRITE TO A FILE!!!) */
  void writeOutIntegralImages(UserDef::VectorOfIntegralImages* integralImages, int classNumber, int height,int width, std::ostream& out){ 

    for(int i=0;i<classNumber;i++){ 

      out<<"Class "<<i<<endl;
      out<<"Of Size "<<integralImages[i].iiVector.size()<<endl;
     
      for(unsigned int s=0;s<integralImages[i].iiVector.size();s++)

	for(int j=0;j<height-1;j++){ 

	  for(int k=0;k<width-1;k++)

	    cout<<((integralImages[i].iiVector.at(s)))[j*width + k]<<" ";
      
	  cout<<endl;
	}

      cout<<endl<<"END OF CLASS ..............."<<endl;
    }
  } 
  

 
 
  /** write the detection windows to a file */ 
  int writeToFile(std::vector<UserDef::WindowClass*> windows, std::ofstream& out){ 
    
    for(unsigned int i=0;i<windows.size();i++)

      out<<"HOffset: "<<windows.at(i)->getY()<<" WOffset: "<<windows.at(i)->getX()<<endl;

    out.close();
    return 0;
  }

  /** write the integral images to a file */  
  int writeToFile(UserDef::VectorOfIntegralImages* integralImages, string writeLoc, int height, int width) {
    
    std::ofstream out;
    
    //   out.open(writeLoc.c_str());

    for(int i=0;i<NCLASSES;i++) {  // For each vector
      
      int p = i; 
      
      char s[50];
      sprintf( s, "%s-class_%d.dat", writeLoc.c_str(), p ); 
      
      
      printf("\n Written down to the file are %zu images of the Class %d. \n", 
	     integralImages[i].iiVector.size(), i );
      
      char writeLocName[ PATH_MAX ]; 
      strcpy( writeLocName, s ); 
      
      out.open( writeLocName ); 
      
      // Write out size
      out<<integralImages[i].iiVector.size()<<endl;

      //Write out contents
      for(unsigned int j=0;j<integralImages[i].iiVector.size();j++)  { // for each ii
	
	int* integralImage = ( integralImages[i].iiVector.at(j) ); 
	
	for(int h=0;h<height;h++)
	  
	  for(int w=0;w<width;w++) 
	    
	    out<<integralImage[h*width + w]<<endl;
	
	
      }

      out.close(); 
      
    }
    //    out.close();
       
    return 0;
  }


  /** read config details from a file */ 
  int readFromFile(UserDef::ConfigClass &config, string readLoc, int height, int width){ 

    // we assume the space for the array of Nclasses is already allocated
    // Clear up vectors

    for( int i=0;i<NCLASSES;i++)
      config.integralImages[i].iiVector.clear();

    //Read the file
    std::ifstream in;

    in.open(readLoc.c_str());

    // temp variables to hold values read from file
    int size;
    int var;
    int* integralImage; 
    
    for(int i=0;i<NCLASSES;i++){ //for each vector

      size = 0; 
      in>>size;
      printf(" Reading %d number of images for the class %d ", size, i ); 

      
      for(int j=0;j<size;j++){
	int ii[height][width];

	for(int h=0;h<height-1;h++)
	  for(int w=0;w<width-1;w++) {
	    in>>var;
	    ii[h][w] = var;
	  }

	integralImage = new int[height*width];

	for(int h=0;h<height-1;h++)
	  for(int w=0;w<width-1;w++)
	    integralImage[h*width + w] = ii[h][w];
	
	config.integralImages[i].iiVector.push_back( integralImage );
      }
    }

    //writeOutIntegralImages(integralImages,NCLASSES, height, width, cout);
    in.close();

    //@false:    cout<<integralImages[0].iiVector.at(0)[23]<<endl;
    return 0;
  }


  /** write config detauils to a file */
  int writeToFile(UserDef::ConfigClass &config){

    //Writing down only necessary parts
    string pathToWrite = CONFIGFILE;
    ofstream out;

    out.open(pathToWrite.c_str());
    
    out<<config.imageInfoInstance.getHeight()<<endl;
    out<<config.imageInfoInstance.getWidth()<<endl;
    out<< config.globalNodeIndex<<endl;
    

    //close file
    out.close();
    return 0;
  }

  /** read config details from a file */ 
  int readFromFile(UserDef::ConfigClass& config){ 

    string pathToRead = CONFIGFILE;

    std::ifstream in;
    in.open(pathToRead.c_str());
    int height;
    in>>height;

    config.imageInfoInstance.setHeight(height);
    int width;
    in>>width;

    config.imageInfoInstance.setWidth(width);
    int globalNodeIndex;
    in>>globalNodeIndex;

    config.globalNodeIndex = globalNodeIndex;

    return 0;
  }
 
  /** print the config files t the console */ 
  int printConfigWindows( UserDef::ConfigClass& config, vector< UserDef::WindowClass* > &windows ) { 

    for( unsigned int i = 0; i < windows.size(); i++ )

      config.globalDetectionsFile
	<< i <<" "
	<<windows.at(i)->getY()
	<<" "
	<<windows.at(i)->getX()
	<<" "
	<<windows.at(i)->getHeight()
	<<" "
	<<windows.at(i)->getWidth()
	<<endl;

    return 0; 
  }

}
