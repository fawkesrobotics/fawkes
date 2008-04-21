#include "UserDef.hh"
#include "Auxillary.hh"
#include <utils/logging/liblogger.h>
/***************************************************************************
 *  UserDef.cpp - Src file for object recognition with random forests: user specific functions, needed only for our framework 
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



namespace UserDef {

  using UserDef::RgbImage;
  using UserDef::WindowClass; 
  
  /** 
   * class to merge detection windows for postprocessing 
   */ 
  class MergeWindowClass : public WindowClass { 
    
  private: 
    /** is the current detection window already considered for detection? */ 
    bool considered ; 
    
  public:
    /** do not consider the current detection window anymore as it was already a part of a previous merge */ 
    void setAsConsidered() { considered = true; } 

    /** is the detection window already considered for postprocessing */    
    bool isConsidered() const { return considered; } 
    
    /**  
     * constructor
     * \param h is the top left coordinate of the detection window (y coordinate) 
     * \param w is the top left coordinate of the detection window (x coordinate) 
     * \param H is the height of the detection window 
     * \param W is the width of the detection window 
     * \param p is the FMP of the detection window 
     */ 
    MergeWindowClass( int h, int w, int H, int W, double p ) : WindowClass( h, w, H, W, p), 
							       considered(false) { }
  }; 
  
  
  /** For a given file location, return that image in the IplImage* format */ 
  IplImage* getImageFromLocation(const string path){
    IplImage* image = 0;

    image = cvLoadImage(path.c_str());

    if(!image){ 
      LibLogger::log_error("UserDef.cpp", "image not found");
      //      perror("UserDef.cpp@getImageFromLocation: image not found!");
      //      exit(1);
    }

    return image;
  }
  

  /** compare 2 integral images */ 
  bool compareIntegralImages( int* ii1, int *ii2, int height, int width )
  {
	
	bool check = true; 
	
	for( int i = 0; i < height-1; i++ )
	{
	  for( int j = 0; j < width-1; j++ )
	  {
		if( ii1[i*width+j] != ii2[i*width+j] )
		{
		  check = false; 
		  return check; 
		}
	  }
	}
	
	return check; 
  }
																	

  /** For a given image at a location, calculate its integralimage */ 
  void calculateIntegralImage(IplImage* image=0, int *integralImage=0){

    const int height = image->height;
    const int width = image->width;

    int ii[height][width];
    int s[height][width];


    //    UserDef::BwImage; 
    //BwImage img(image);
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
	  //	  cout<<"lesser than 0";
	  //	  exit(0);
	}
	else
	  integralImage[i*width + j] = ii[i][j]; 
    
  }
  
  /** get the rectangular feature score */ 
  double getRectangularIntegralImagefeature(int* integralImageCalculated, int xMain, int yMain, int x, int y, int height, int width, int haarFeature){
    
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
    //    cout<<"UserDef.cpp:getRec*: "<<normalizedII;
    return normalizedII;
  }


  /**  this merge detections is based on filterNoise2, that is merging based on areas 
   // Most important part of the program  */ 
  void mergeDetections( std::vector<UserDef::WindowClass*> &windows, ConfigClass& config , 
			double scaleFactor ) { 


    int r1_lx, r1_ly, r1_rx, r1_ry, r2_lx, r2_ly, r2_rx, r2_ry; 
    r1_lx = r1_ly = r1_rx = r1_ry = r2_lx = r2_ly = r2_rx = r2_ry = 0; 
    
    /*
     * the percentage of an overlapping window that is considered a part of another detection hit: 
     * i.e. merge if this much perctange of area overlaps.
     */ const double area_merge_ratio = AREA_OF_SUBWINDOW_MERGE;
    
    
    vector< MergeWindowClass* > final, detections, chosen; 
    
    /* 
     * Part of the old code 
     */    //     for( int i = 0; i < windows.size(); i++ )  
//       detections.push_back( new MergeWindowClass( windows.at(i)->getY(), 
// 						  windows.at(i)->getX(),
// 						  windows.at(i)->getHeight(),
// 						  windows.at(i)->getWidth(),
// 						  windows.at(i)->getFaceProbabilityResult() ) );



    /*
     * *******************
     * NEED TO SORT the probabilities so that they are considere from hightest probabiluty to lowest probability
     * due to the price of merging procedures
     * TODO!!!! 
     ********************
     */ 
    

    /* 
     * Begin 'bad' sort section 
     */    
    double face_probability[ windows.size() ]; 


    for( unsigned int i=0; i<windows.size(); ++i ) 
      face_probability[ i ] = windows.at(i)->getFaceProbabilityResult(); 

    std::sort( face_probability, face_probability + detections.size() ); 

    // temp place to store windows 
    vector< UserDef::WindowClass* > tempwindows;
    for( unsigned int i = 0 ; i < windows.size() ; i++ ) 
      tempwindows.push_back( windows.at ( i ) ); 

    for( int i= windows.size()-1; i!=-1; i-- ) { 

      for( unsigned int j=0; j<tempwindows.size() ; j++) 
	if( tempwindows.at(j) != 0 )
	  if( tempwindows.at(j)->getFaceProbabilityResult() == face_probability[i] ) { 
	    
	    detections.push_back( new MergeWindowClass( tempwindows.at(j)->getY(),
							tempwindows.at(j)->getX(),
							tempwindows.at(j)->getHeight(),
							tempwindows.at(j)->getWidth(),
							tempwindows.at(j)->getFaceProbabilityResult() ) );
	    
	    tempwindows.at(j) = 0 ; 
	    break; 
	    
	  }
      
    } 
    /*
     * End 'bad' sorting section
     */
    
    


    // the code below is constructed with the assumption that all the rectangles are of the same size
    // it is mergedetections from the same scale after all

    vector< MergeWindowClass* > temp; 

    for( unsigned int i = 0; i < detections.size(); i++ ) { 
      
      if( detections.at(i)->isConsidered() ) 
	continue; 
      
      MergeWindowClass *r1 = detections.at(i); 

      /*
       * for each subwindow, I am going to consider all "neighbors", mark them all, and choose the best
       *  temp will store them. Put all 'to be considered as neighbors' in one set (temp). 
       */ temp.push_back( r1 );
      
      for( unsigned int j = 0; j < detections.size(); j++ ) { 

	if( detections.at(j)->isConsidered() || j==i ) // either already considered or it is the same
	  continue; 

	MergeWindowClass *r2 = detections.at(j); //r2 is the other rectangle 


	r1_lx = r1->getX(); r1_ly = r1->getY();
	r1_rx = r1->getX() + r1->getWidth(); r1_ry = r1->getY() + r1->getHeight(); 

	r2_lx = r2->getX(); r2_ly = r2->getY(); 
	r2_rx = r2->getX() + r2->getWidth(); r2_ry = r2->getY() + r2->getHeight(); 



	int areaOfR2 = r2->getHeight() * r2->getWidth();
	//	int areaOfR1 = r1->getHeight() * r1->getWidth(); 

	// if r2 falls into any of teh below conditions, then it iwll be put into temp and the best one will be chosen
	
	// case1: r2 is topleft corner of r1
	if( r2_lx <= r1_lx && 
	    r2_ly <= r1_ly && 
	    r2_rx <= r1_rx && 
	    r2_ry <= r1_ry ) { // is overlapping area passing threshold 

	  int area_of_overlap = (r1_lx - r2_lx) * r2->getHeight() + (r1_ly - r2_ly) * r2->getWidth() 
	    - (r1_lx - r2_lx) * (r1_ly - r2_ly); 

	  if( (double)area_of_overlap/areaOfR2 >= area_merge_ratio ) // consider this
	    temp.push_back( r2 ); 

	} 
	
	// case 2: r2 is top right of r1
	else if( r2_lx >= r1_lx && 
		 r2_ly <= r1_ly && 
		 r2_rx >= r1_rx && 
		 r2_ry <= r1_ry ) { 

	  int area_of_overlap = (r1_ly - r2_ly) * r2->getWidth() + (r2_rx - r1_rx) * r2->getHeight()  - 
	    (r2_rx - r1_rx) * (r1_ly - r2_ly ); 

	  if( (double)area_of_overlap/areaOfR2 >= area_merge_ratio ) //consider this
	    temp.push_back( r2 ); 

	}


	// case 3: bottom left corner
	else if( r2_lx <= r1_lx && 
		 r2_ly >= r1_ly && 
		 r1_rx >= r2_rx && 
		 r2_ry >= r1_ry ) { 

	  int area_of_merge = r2->getHeight() * (r1_lx - r2_lx ) + 
	    r2->getWidth() * (r2_ry - r1_ry) - (r1_lx - r2_lx)*(r2_ry - r1_ry); 

	  if( (double)area_of_merge/areaOfR2 >= area_merge_ratio ) 
	    temp.push_back( r2 ); 

	}

	//case 4": bottomr ight corner
	else if( r2_lx >= r1_lx && 
		 r2_ly >= r1_ly && 
		 r2_rx >= r1_rx && 
		 r2_ry >= r1_ry ) { 
	  
	  int area_of_merge = r2->getHeight() * ( r2_rx - r1_rx ) +
	    r2->getWidth() * (r2_ry - r1_ry ) - 
	    (r2_rx - r1_rx ) * ( r2_ry - r1_ry); 

	  if( (double)area_of_merge/areaOfR2 >= area_merge_ratio ) 
	    temp.push_back( r2 ); 

	}

       }
	
      /*
       * All neighbors are in temp. Pick the best. 
       */ double pmax = -2.0;       
      int index = -1; 
      
      for( unsigned int j = 0; j < temp.size() ; j++ ) { 
	
	if( temp.at(j)->getFaceProbabilityResult() >= pmax ) { 
	  pmax = temp.at(j)->getFaceProbabilityResult(); 
	  index = j ; 
	}

	temp.at(j)->setAsConsidered();  // In any case, all must be NOT considered again (considered flag should be set)

      }
      
      if( temp.size() != 0 ) 
	chosen.push_back( temp.at( index ) ); //this has the chosen one 


      temp.clear(); 

    }
	
    
    WindowClass* wc = 0 ;
    for( unsigned int i = 0 ; i < windows.size() ; i++ ) { 
      wc = windows.at( i ) ;
      delete wc; 
    }
    
    windows.clear();
    
      for( unsigned int i = 0; i < chosen.size(); i++ )  
	  windows.push_back( new WindowClass( chosen.at(i)->getY(),
					      chosen.at(i)->getX(),
					      chosen.at(i)->getHeight(),
					      chosen.at(i)->getWidth(),
					      chosen.at(i)->getFaceProbabilityResult() ) );
	  
      

      chosen.clear(); 
       
      MergeWindowClass* mwc = 0 ;
      for( unsigned int i = 0 ; i < detections.size() ; i++ ) { 
	mwc = detections.at( i ); 
	delete mwc; 
      }
      detections.clear(); 
 
  }
	
    
  
  
  /** a not so effective but quick merging scheme heuristic */ 
  void mergeDetections_old( std::vector<UserDef::WindowClass*> &windows, ConfigClass& config , 
			double scaleFactor ) { 
    
    vector< MergeWindowClass* > temp, detections, final;
    
    temp.clear();
    detections.clear();
    final.clear();
    
    /* 
     * Decides the range between detection rectangles where detection hits are considered equivalent 
     * It Chooses the minimum of height/width of the training images
     */

//    int neighborRange = WINDOWS_INTEGRATION_THRESHOLD;
    int neighborRange = (int) ( scaleFactor * 
				( 
				 config.imageInfoInstance.getHeight() > config.imageInfoInstance.getWidth()? 
				 config.imageInfoInstance.getWidth(): config.imageInfoInstance.getHeight() 
				 ) 
				); 

    neighborRange /= 3; // Keeping neighbor range as is results is LARGE gaps between detections

    // First put ALL detections in wrapper class (that is able to denote detections already considered)
    for( unsigned int i = 0; i < windows.size(); i++ ) 
      detections.push_back( new MergeWindowClass( windows.at(i)->getY(), 
						  windows.at(i)->getX(), 
						  windows.at(i)->getHeight(), 
						  windows.at(i)->getWidth(),
						  windows.at(i)->getFaceProbabilityResult() ) );
    
    
    for( unsigned int i = 0; i < detections.size(); i++ ) { 
      
      MergeWindowClass* chosenOne = detections.at(i); 
      using Auxillary::inRange;
      
      if( !chosenOne->isConsidered() ) { 

	temp.push_back( chosenOne ); 
	chosenOne->setAsConsidered(); 
	
	for( unsigned int j = 0; j < detections.size(); j++ ) { 
	  
	  /* 
	   * The general procedure if any other detection hit comes just as close to the current one, then lock them both out
	   * and keep only one. 
	   */
	  
	  if( j!=i && 
	      !detections.at(j)->isConsidered() && 
	      inRange( detections.at(j)->getX(), chosenOne->getX(), neighborRange ) && 
	      inRange( detections.at(j)->getY(), chosenOne->getY(), neighborRange ) 
	      ) { 
	    
	    temp.push_back( detections.at(j) ); 
	    detections.at(j)->setAsConsidered(); 
	  } 
	}  
	
      double highestProbability = -1.0;
      int indexOfHighest = -1; 

      bool average = true;  // Start averaging bounding box technqiue for the neighbors: a bad show
      
      if( !average) {  
	
	// The alternative: pick the one with the highest FMP val. 
	for( unsigned int j = 0; j < temp.size(); j++ ) 	  
	  if( temp.at(j)->getFaceProbabilityResult() > highestProbability || 
	      temp.at(j)->getFaceProbabilityResult() == highestProbability ) { 
	    
	    highestProbability = temp.at(j)->getFaceProbabilityResult(); 
	    indexOfHighest = j; 
	  }
	
	if( temp.size()!=0 )
	  final.push_back( temp.at( indexOfHighest ) ); 
	
      } // if( !ave.. 
      
      else { 
	int avgX = 0 , avgY = 0;
	double avgP = 0.0F; 
	
	for( unsigned int j = 0; j<temp.size(); j++) { 

	  avgX += temp.at(j)->getX();
	  avgY += temp.at(j)->getY(); 
	  avgP += temp.at(j)->getFaceProbabilityResult(); 
	}
	

	if( temp.size()!=0 ) { 
	  avgX /= temp.size();
	  avgY /= temp.size();
	  avgP /= temp.size(); 

	  final.push_back( new MergeWindowClass( avgY, 
						 avgX, 
						 temp.at(0)->getHeight(), 
						 temp.at(0)->getWidth(), 
						 avgP ) );
	} 
	
	} 
      
      temp.clear(); 
      
      } // if( !cho..
      
    } // for( i..
    
    for( unsigned int i = 0; i < detections.size(); i++ )  // For those that have no neighbors
      if( !detections.at(i)->isConsidered() )
	final.push_back( detections.at(i) ); 
    
    WindowClass* wc = 0;
    for( unsigned int i = 0 ; i < windows.size() ; i++ ) { 
      wc = windows.at( i );
      delete wc; 
    }
      
    windows.clear(); 
    
    for( unsigned int i = 0; i < final.size(); i++ ) 
      windows.push_back( new WindowClass( final.at(i)->getY(), 
					  final.at(i)->getX(), 
					  final.at(i)->getHeight(),
					  final.at(i)->getWidth(),
					  final.at(i)->getFaceProbabilityResult() ) );
    
      

    MergeWindowClass* mwc = 0; 
    for( unsigned int i =  0 ; i < final.size() ; i++ ) { 
      mwc = final.at(i);
      delete mwc; 
    }
    final.clear();

    for( unsigned int i = 0 ; i < detections.size() ; i++ ) { 
      mwc = detections.at( i );
      delete mwc; 
    }
    detections.clear(); 


  } // End mergeDections( ..

  
 //  /** merge detectin windows */ 
//   void mergeDetectionSubwindow(std::vector<UserDef::WindowClass*> &windows, int height, int width, ConfigClass& config){ // 1. lets use a temp WindowClass vector to store those in one set

// //     class MergeWindowClass : public WindowClass { 
      
// //       bool merged; 

// //       MergeWindowClass( int y, int x, int H, int W) : WindowClass( y, x, H, W), 
// // 						      merged( false) {}

// //       bool isMerged() const { return merged; } 
      
// //       void merge() { merged = true; } 

//     };

//     // tempSet will have the new set of patches
//     vector<UserDef::WindowClass*> tempSet;
//     tempSet.clear();
//     // These include the windows already considered
//     vector<UserDef::WindowClass*> alreadyConsidered;
//     alreadyConsidered.clear();

//        // declare vector of merge subwindows
//     std::vector<MergeWindowClass*> mergeWindows;
//     mergeWindows.clear();
//     for(int i=0;i<windows.size();i++)
//       mergeWindows.push_back(new MergeWindowClass(windows.at(i)->getY(), windows.at(i)->getX(), windows.at(i)->getHeight(), windows.at(i)->getWidth()));
    
//     int hOffset, wOffset;
//     double faceProbabilityResult = -1.0;
//     int heightOfSubWindow;
//     int widthOfSubWindow;
//     int mergeOption = 1; 
//     // 2. lets start with the first of its kind
//     for(int i=0;i<mergeWindows.size();i++)
//       if(!(mergeWindows.at(i)->isMerged())){ 
//       //2. lets record x and y coordinates and height and width
// 	heightOfSubWindow = mergeWindows.at(i)->getHeight();
// 	widthOfSubWindow = mergeWindows.at(i)->getWidth();
// 	faceProbabilityResult = mergeWindows.at(i)->getFaceProbabilityResult();
// 	hOffset = mergeWindows.at(i)->getY();
// 	wOffset = mergeWindows.at(i)->getX();
// 	//3. Now list all neighbors - controlled by OVERLAPPING_BORDERS
//       int counter = 1;
//       //4. I comment the more efficient code and instead consdier all possible subwindows apart from the one in the loop considered above

//       for(int j=0;j<mergeWindows.size();j++){ 
// 	if((hOffset+config.getDetectionThreshold()<mergeWindows.at(j)->getY()) && (wOffset+config.getDetectionThreshold()<mergeWindows.at(j)->getX())) 
// 	  if(!(mergeWindows.at(j)->isMerged()) && j!=i){ 
// 	    hOffset += mergeWindows.at(j)->getY();
// 	    wOffset += mergeWindows.at(j)->getX();
// 	    heightOfSubWindow += mergeWindows.at(j)->getHeight();
// 	    widthOfSubWindow += mergeWindows.at(j)->getWidth();
// 	    mergeWindows.at(j)->merge();
// 	    cout<<"index of j - subwindow chosen to merge"<<j<<endl;
// 	    counter++;
// 	  }
//       }

//       //      for(int j=i+1;j<mergeWindows.size() && (mergeWindows.at(j)->getHeight()<(hOffset+OVERLAPPING_BORDERS) && mergeWindows.at(j)->getWidth()<(wOffset+OVERLAPPING_BORDERS)) && !(mergeWindows.at(j)->isMerged());j++) { 
// 	// Merge option simply merges every subwindow is that region - may be too harsh - the end result are only a small number of subwindows. 
// // 	if(mergeOption ==1){ 
// // 	hOffset += mergeWindows.at(j)->getHeight();
// // 	wOffset += mergeWindows.at(j)->getWidth(); 
// // 	mergeWindows.at(j)->merge();
// // 	counter++;
// // 	}
// 	// //Merge option checks if they actually share overlapping boundaries and merges them only then 
// // 	if(mergeOption ==2){ 
// // 	  if(windows.at(j)->getHeight()>hOffset && windows.at(j)->getHeight()<(hOffset + height)) { 
// // 	    //We now have over lapping windows - merge them 
// // 	  }
//  //      }
// //       }

//     // Consider averages of hOffsets and wOffsets 
//       if(counter!=0) { 
// 	hOffset /= counter;
// 	wOffset /=counter;
// 	heightOfSubWindow /= counter;
// 	widthOfSubWindow /= counter;
// 	faceProbabilityResult /= counter;
// 	cout<<"Counter"<<counter<<" hOffset and wOffset "<<hOffset<<"  "<<wOffset<<endl;
//       }
//       // Put this new window in this vector
// //       tempSet.push_back(new UserDef::WindowClass(hOffset, wOffset, heightOfSubWindow, widthOfSubWindow, faceProbabilityResult));
// //     }
// //     windows.clear();
// //     windows.assign(tempSet.begin(),  tempSet.end());
// }

/** draw detection results on the image */ 
  void drawDetections(IplImage* image, vector<UserDef::WindowClass*> windows, string nameLocOfImage, char*  nameAdd, bool saveImages ){ 
    
    // Make sure image exists
    if(!image){ 
      LibLogger::log_error("Userdef.cpp","no image present"); 
      //      perror("UserDef.cpp@drawDetections: There is no image!");
      //      exit(1);
    }

    char fullname[PATH_MAX];
    char* filename;

    char detfilename[PATH_MAX];
    char detname[] = "det-";

    strcpy(fullname, nameLocOfImage.c_str());
    filename = strrchr( fullname, '\\' );
    if( filename == NULL )
    {
        filename = strrchr( fullname, '/' );
    }
    if( filename == NULL )
    {
        filename = fullname;
    }
    else
    {
        filename++;
    }

    


    bool drawCircles = false;
    static CvScalar colors[] = 
      {
        {{0,0,255}},
        {{0,128,255}},
        {{0,255,255}},
        {{0,255,0}},
        {{255,128,0}},
        {{255,255,0}},
        {{255,0,0}},
        {{255,0,255}}
      };
    
    int scale = 1;
    
    // Create an actual Window
    //cvNamedWindow("result", 1);
    UserDef::WindowClass* theWindow;
    
    if(drawCircles)
    for(unsigned int i=0;i<windows.size();i++){ 
      theWindow = windows.at(i);
      int height = theWindow->getHeight();
      int width = theWindow->getWidth();
      CvPoint center;
      int radius;
      center.x = cvRound(theWindow->getX() + width/2);
      center.y = cvRound(theWindow->getY() + height/2);
      radius = cvRound(height/2);
      cvCircle(image, center, radius, colors[i%8], 3, 8, 0);
    }

    else{ 
      CvPoint pt1, pt2; 
      for(unsigned int i=0;i<windows.size();i++){ 
	theWindow = windows.at(i);

	int height = theWindow->getHeight();
	int width = theWindow->getWidth();

	pt1.x = (theWindow->getX())*scale;
	pt2.x = (theWindow->getX() + width)*scale;
	pt1.y = (theWindow->getY())*scale;
	pt2.y = (theWindow->getY() + height)*scale;

	cvRectangle(image, pt1, pt2, colors[i%8], 3, 8,0);
      }
    }

   
    //    cvShowImage("result", image);
    
    strcpy(detfilename, detname);
    strcat(detfilename, nameAdd);

    strcat(detfilename, filename);
//     strcpy(filename, detfilename);
//     cvvSaveImage(fullname, image);

    if(saveImages) cvvSaveImage(detfilename, image);
    // cvWaitKey(4);
    // cvDestroyWindow("result");
  }
 

  /** heuristic 1 for merging detections */ 
  void filterNoise1( ConfigClass &config, vector< WindowClass* > &windows) { 
    
    vector< WindowClass* > final;
    
    for( unsigned int i=0; i < windows.size(); i++ ) { 

      WindowClass* r1 = windows.at(i); 
      int flag = 1;

      for( unsigned int j=0; j < windows.size(); j++ ) { 

	WindowClass* r2 = windows.at(j); 

	int distance = cvRound( r2->getWidth() * 0.2 ); 

	if( i!=j && 
	    r1->getX() >= r2->getX() - distance && 
	    r1->getY() >= r2->getY() - distance && 
	    r1->getX() + r1->getWidth() <= r2->getX() + r2->getWidth() + distance && 
	    r2->getY() + r2->getHeight() <= r2->getY() + r2->getHeight() + distance ) { 
	 
	  flag = 0;
	  break;
	}
      }

      if( flag ) 
	final.push_back( new WindowClass( r1->getY(), r1->getX(), r1->getHeight(), r1->getWidth(), 
					  r1->getFaceProbabilityResult() ) ); 

    }

    windows.clear();
    
    for( unsigned int i = 0; i < final.size() ; i++ ) 
      windows.push_back( final.at( i ) ); 

  }	    

  /** heuristic type 2 */
  void filterNoise2( ConfigClass& config, vector< WindowClass* >& windows) { 
    
    // Remove all the inner rectangles 
    int topLeftX = 0, topLeftY = 0, bottomRightX = 0, bottomRightY = 0;
    
    
    // the percentage of an overlapping window that is considered a part of another detection hit: 
    // i.e. merge if this much perctange of area overlaps.
    const double area = AREA_OF_SUBWINDOW_MERGE; 
    
    vector< MergeWindowClass* > temp, final, detections; 
    
    
    /*
     * Might need to make some modifications here like the ones i did for mergeDetection
     * maybe include a getFaceProbabilityResult() based sorting and then carry out below
     * 
     * SORT getFaceProbabilityResult() 
     *
     */ 


    for( unsigned int i = 0; i < windows.size(); i++ )  
      detections.push_back( new MergeWindowClass( windows.at(i)->getY(), 
						  windows.at(i)->getX(),
						  windows.at(i)->getHeight(),
						  windows.at(i)->getWidth(),
						  windows.at(i)->getFaceProbabilityResult() ) );
    
    for( unsigned int i = 0; i < windows.size(); i++ ) { 
      
      MergeWindowClass* r1 = detections.at( i );
      
      if( !r1->isConsidered() ) { 
	
	topLeftX = r1->getX();
	topLeftY = r1->getY(); 
	bottomRightX = r1->getX() + r1->getWidth(); 
	bottomRightY = r1->getY() + r1->getHeight();
	
	
	for( unsigned int j = 0; j < windows.size(); j++ ) { 
	  
	  MergeWindowClass* r2 = detections.at( j );

	  int areaOfR2 = r2->getHeight() * r2->getWidth(); 
	  
	  // Case 1: The r2 is completely inside r1
	  if( j!=i && 
	      !r2->isConsidered() && 
	      r2->getX() >= topLeftX && 
	      r2->getX() + r2->getWidth() <= bottomRightX && 
	      r2->getY() >= topLeftY && 
	      r2->getY() + r2->getHeight() <= bottomRightY )  
	    r2->setAsConsidered(); 

	  // Case 2: the width of r2 is inside r1
	  else if( j!=i && 
		   !r2->isConsidered() && 
		   r2->getX() >= topLeftX && 
		   r2->getX() + r2->getWidth() <= bottomRightX ) { 
	    
	    // Two cases: The smaller rectangle is atleast 20% inside from top
	    if( r2->getY() + (int)( (1 - area) * r2->getHeight() ) >= topLeftY &&
		r2->getY() <= topLeftY && 
		r2->getY() + r2->getHeight() <= bottomRightY ) 
	      r2->setAsConsidered();

	    //Other case
	    if( r2->getY() + (int)( area * r2->getHeight() ) <= bottomRightY && 
		r2->getY() <= bottomRightY && 
		r2->getY() >= topLeftY )
	      r2->setAsConsidered(); 

	  } // else if(..

	  // Case 3: the height of r2 is inside r2
	  else if( j!=i && 
		   !r2->isConsidered() && 
		   r2->getY() >= topLeftY && 
		   r2->getY() + r2->getHeight() <= bottomRightY ) { 

	    if( r2->getX() + (int)( (1-area) * r2->getWidth() ) >= topLeftX && 
		r2->getX() <= topLeftX && 
		r2->getX() + r2->getWidth() <= bottomRightX ) 
	      r2->setAsConsidered(); 
	    
	    if( r2->getX() >= topLeftX && 
		r2->getX() + (int)( area * r2->getWidth() ) <= bottomRightX && 
		r2->getX() <= bottomRightX ) 
	      r2->setAsConsidered(); 
	  } // else if(..

	  
	  // Case 4: If just a portion is within the bigger rectanlge widthwise
	  else if( j!=i && 
		   !r2->isConsidered() && 
		   ( r2->getY() <= topLeftY && r2->getY() + r2->getHeight() <= bottomRightY ) ) { 

	    // Case a: the portion is jutting out of the right of the bigger rectange
	    if( r2->getX() >= topLeftX && r2->getX() + r2->getWidth() >= bottomRightX 
		&& r2->getX() <= bottomRightX ) { 
	      int areaOfOverlap = ( r2->getY() + r2->getHeight() - topLeftY ) * 
		( r2->getX() + r2->getWidth() - bottomRightX ); 
	      
	      double perctangeOverlap = (double)areaOfOverlap / (double)areaOfR2; 
	      
	      if( perctangeOverlap > area ) 
		r2->setAsConsidered(); 

	    } // Case a..

	    // Case b: the portion jutting out of the left of the bigger rectabgel 
	    else if( r2->getX() <= topLeftX && r2->getX() + r2->getWidth() >= topLeftX 
		     && r2->getX() + r2->getWidth() <= bottomRightX ) { 
	      
	      int areaOfOverlap = ( r2->getY() + r2->getHeight() - topLeftY) * 
		( r2->getX() + r2->getWidth() - topLeftX ); 

	      double perctangeOverlap = (double)areaOfOverlap / (double)areaOfR2; 
	      
	      if( perctangeOverlap > area ) 
		r2->setAsConsidered(); 
	    } // Case b .. 


	    // Case c: the portion is jutting out of left and right of the bigger rectangle 
	    else if( r2->getX() <= topLeftX && r2->getX() + r2->getWidth() >= bottomRightX ) { 

	      int areaOfOverlap = ( r2->getY() + r2->getHeight() - topLeftX ) * 
		( r2->getWidth() - bottomRightX  + topLeftX ); 

	      double perctangeOverlap = (double)areaOfOverlap / (double)areaOfR2; 
	      
	      if( perctangeOverlap > area )
		r2->setAsConsidered(); 

	    } // Case c..

	  } // Case 4.. 	  
	  
	} 
      }
    }
    
    
    WindowClass* wc = 0 ;
    for( unsigned int i = 0 ; i < windows.size() ; i++ ) { 
      wc = windows.at( i ) ;
      delete wc; 
    }
    
    windows.clear();
    
      for(unsigned int i = 0; i < detections.size(); i++ )  
	if( !detections.at(i)->isConsidered() ) 
	  windows.push_back( new WindowClass( detections.at(i)->getY(),
					      detections.at(i)->getX(),
					      detections.at(i)->getHeight(),
					      detections.at(i)->getWidth(),
					      detections.at(i)->getFaceProbabilityResult() ) );
	  
      

  
      
      MergeWindowClass* mwc = 0 ;
      for( unsigned int i = 0 ; i < detections.size() ; i++ ) { 
	mwc = detections.at( i ); 
	delete mwc; 
      }
      detections.clear(); 
      

    }
	

  /** heuristic type 3 */ 
  void filterNoise3( std::vector<UserDef::WindowClass*> &windows, ConfigClass& config , 
					double scaleFactor ) { 
    
    
    int r1_lx, r1_ly, r1_rx, r1_ry, r2_lx, r2_ly, r2_rx, r2_ry; 
    r1_lx = r1_ly = r1_rx = r1_ry = r2_lx = r2_ly = r2_rx = r2_ry = 0; 
    
    /*
     * the percentage of an overlapping window that is considered a part of another detection hit: 
     * i.e. merge if this much perctange of area overlaps.
     */ const double area_merge_ratio = AREA_OF_SUBWINDOW_MERGE;
    
    
    vector< MergeWindowClass* > final, detections, chosen; 
    
    /* 
     * Part of the old code 
     */    //     for( int i = 0; i < windows.size(); i++ )  
	//       detections.push_back( new MergeWindowClass( windows.at(i)->getY(), 
	// 						  windows.at(i)->getX(),
	// 						  windows.at(i)->getHeight(),
	// 						  windows.at(i)->getWidth(),
	// 						  windows.at(i)->getFaceProbabilityResult() ) );
	
	
	
    /*
     * *******************
     * NEED TO SORT the probabilities so that they are considere from hightest probability to lowest probability
     * due to the price of merging procedures
     * TODO!!!! 
     ********************
     */ 
    
	
    /* 
     * Begin 'bad' sort section 
     */    
    double face_probability[ windows.size() ]; 
	
	
    for( unsigned int i=0; i<windows.size(); ++i ) 
      face_probability[ i ] = windows.at(i)->getFaceProbabilityResult(); 
	
    std::sort( face_probability, face_probability + detections.size() ); 
	
    // temp place to store windows 
    vector< UserDef::WindowClass* > tempwindows;
    for( unsigned int i = 0 ; i < windows.size() ; i++ ) 
      tempwindows.push_back( windows.at ( i ) ); 
    
    for( int i= windows.size()-1; i!=-1; i-- ) { 
      
      for( unsigned int j=0; j<tempwindows.size() ; j++) 
		if( tempwindows.at(j) != 0 )
		  if( tempwindows.at(j)->getFaceProbabilityResult() == face_probability[i] ) { 
			
			detections.push_back( new MergeWindowClass( tempwindows.at(j)->getY(),
													   tempwindows.at(j)->getX(),
													   tempwindows.at(j)->getHeight(),
													   tempwindows.at(j)->getWidth(),
													   tempwindows.at(j)->getFaceProbabilityResult() ) );
			
			tempwindows.at(j) = 0 ; 
			break; 
			
		  }
      
    } 
    /*
     * End 'bad' sorting section : but seems to work
     */
    
    tempwindows.clear(); 
    
    
    // the code below is constructed with the assumption that all the rectangles are of the same size
    // it is mergedetections from the same scale after all
    
    vector< MergeWindowClass* > temp; 
    
    for( unsigned int i = 0; i < detections.size(); i++ ) { 
      
      if( detections.at(i)->isConsidered() ) 
		continue; 
      
      MergeWindowClass *r1 = detections.at(i); 
      
      /*
       * for each subwindow, I am going to consider all "neighbors", mark them all, and choose the best
       *  temp will store them. Put all 'to be considered as neighbors' in one set (temp). 
       */ 
      
      temp.push_back( r1 ); //: I am going to do this later on 
	  
	  
      bool isNeighbor = false; 
      
      for( unsigned int j = 0; j < detections.size(); j++ ) { 
		
		if( detections.at(j)->isConsidered() || j==i ) // either already considered or it is the same
		  continue; 
		
		MergeWindowClass *r2 = detections.at(j); //r2 is the other rectangle 
		
		bool swapped = false; 
		
		/*
		 * ********* 
		 * Since it can happen that the smaller rectangle is r1, I am going to try and see if I can swtich them
		 * i.e. the bigger rectanlge is always r1 : so a simple swap
		 */
		
		if( r2->getHeight() * r2->getWidth() > r1->getHeight() * r1->getWidth() ){ 
	      MergeWindowClass* tempRectangle = r1;
	      r1 = r2;
	      r2 = tempRectangle;
	      tempRectangle = 0; 
	      swapped = true; 
	    }
		
		r1_lx = r1->getX(); r1_ly = r1->getY();
		r1_rx = r1->getX() + r1->getWidth(); r1_ry = r1->getY() + r1->getHeight(); 
		
		r2_lx = r2->getX(); r2_ly = r2->getY(); 
		r2_rx = r2->getX() + r2->getWidth(); r2_ry = r2->getY() + r2->getHeight(); 
		
		
		
		int areaOfR2 = r2->getHeight() * r2->getWidth();
		//		int areaOfR1 = r1->getHeight() * r1->getWidth(); 
		
		// if r2 falls into any of teh below conditions, then it iwll be put into temp and the best one will be chosen
		
		// case1: r2 is topleft corner of r1
		if( r2_lx <= r1_lx && 
		   r2_ly <= r1_ly && 
		   r2_rx <= r1_rx && 
		   r2_ry <= r1_ry  && 
		   r2_rx >= r1_lx && 
		   r2_ry >= r1_ly 
		   ) { // is overlapping area passing threshold 
		  
		  int area_of_overlap; 
		  
		  // I seem to have wrongly reasoned that there are 2 cases: but there is only one isn't there?
		  
		  // 	  // maximum portion is outside : get outside area
		  // 	  int area_of_overlap = (r1_lx - r2_lx) * r2->getHeight() + (r1_ly - r2_ly) * r2->getWidth() 
		  // 	    - (r1_lx - r2_lx) * (r1_ly - r2_ly); 
		  
		  // 	  area_of_overlap = areaOfR2 - area_of_overlap; 
		  
		  // 	  if( (double)area_of_overlap/areaOfR2 >= area_merge_ratio ) // consider this
		  // 	    isNeighbor = true; 
		  
		  // 	  else { 
		  area_of_overlap = (r2_rx - r1_lx) * (r2_ry - r1_ly); 
		  
		  // simple check : redundant
		  if( area_of_overlap < 0 ) { 
			perror("1: area of overlap < 0 "); 
			//	      exit( 1 ); 
		  }
		  
		  if( (double)area_of_overlap/areaOfR2 >=area_merge_ratio) 
			isNeighbor = true;
		  //	  }
		  
		} 
		
		// case 2: r2 is top right of r1
		else if( r2_lx >= r1_lx && 
				r2_ly <= r1_ly && 
				r2_rx >= r1_rx && 
				r2_ry <= r1_ry && 
				r2_lx <= r1_rx && 
				r2_ry >= r1_ly ) { 
		  
		  int area_of_overlap; 
		  
		  // 	  int area_of_overlap = (r1_ly - r2_ly) * r2->getWidth() + (r2_rx - r1_rx) * r2->getHeight()  - 
		  // 	    (r2_rx - r1_rx) * (r1_ly - r2_ly ); 
		  
		  
		  // 	  // if outer area
		  // 	  if( (double)area_of_overlap/areaOfR2 >= area_merge_ratio ) //consider this
		  // 	    isNeighbor = true;
		  
		  // 	  else { 
		  
		  area_of_overlap = (r2_ry - r1_ly) * ( r1_rx - r2_lx); 
		  
		  
		  if( area_of_overlap < 0 ) { 
			perror("2: area of overlap error");
			//	    exit( 1 );
		  }
		  
		  if( (double)area_of_overlap/areaOfR2 >= area_merge_ratio ) 
			isNeighbor  = true;
		  
		  // 	  }
		  
		  
		}
		
		
		// case 3: bottom left corner
		else if( r2_lx <= r1_lx && 
				r2_ly >= r1_ly && 
				r1_rx >= r2_rx && 
				r2_ry >= r1_ry && 
				r2_rx >= r1_lx && 
				r2_ly <= r2_ry ) { 
		  
		  int area_of_overlap = 0; 
		  
		  // 	  int area_of_overlap = r2->getHeight() * (r1_lx - r2_lx ) + 
		  // 	    r2->getWidth() * (r2_ry - r1_ry) - (r1_lx - r2_lx)*(r2_ry - r1_ry); 
		  
		  // 	  if( (double)area_of_overlap/areaOfR2 >= area_merge_ratio ) 
		  // 	    isNeighbor = ( true );
		  
		  // 	  else { 
		  area_of_overlap = (r2_rx - r1_lx) * ( r2_ry - r1_ly); 
		  
		  if( area_of_overlap < 0 ) { 
			perror("3: area of overlap error");
			//	      exit( 1 );
		  }
		  
		  if( (double)area_of_overlap/areaOfR2 >= area_merge_ratio ) 
			isNeighbor = ( true );
		  
		  //	  }
		  
		}
		
		//case 4": bottomr ight corner
		else if( r2_lx >= r1_lx && 
				r2_ly >= r1_ly && 
				r2_rx >= r1_rx && 
				r2_ry >= r1_ry && 
				r2_lx <= r1_rx && 
				r2_ly <= r1_ry
				) { 
		  
		  int area_of_overlap; 
		  
		  // 	  int area_of_overlap = r2->getHeight() * ( r2_rx - r1_rx ) +
		  // 	    r2->getWidth() * (r2_ry - r1_ry ) - 
		  // 	    (r2_rx - r1_rx ) * ( r2_ry - r1_ry); 
		  
		  // 	  if( (double)area_of_overlap/areaOfR2 >= area_merge_ratio ) 
		  // 	    isNeighbor = ( true );
		  
		  // 	  else { 
		  
		  area_of_overlap = (r1_rx - r2_lx ) * ( r1_ry - r2_ly ); 
		  if( area_of_overlap < 0 ) { 
			perror(" 4:area of overlap ");
			//	      exit( 1 ); 
		  }
		  
	      
		  
		  if( (double)area_of_overlap/areaOfR2 >= area_merge_ratio )
			isNeighbor = ( true );
		  //	  }
		  
		}
		
		
		//case 5: box is completely inside bigger box
		else if( r2_lx >= r1_lx && 
				r2_ly >= r1_ly && 
				r2_rx <= r1_rx && 
				r2_ry <= r1_ry ) // no need to check areas
		  isNeighbor = ( true ); 
		
		
		
		//case 6: the small rectangle is inside the region of the bigger one, but a small portion juts out ( that is, its not completely inside) [ at the bottom of the bigger rectange ] 
		else if( r2_lx >= r1_lx && 
				r2_rx <= r1_rx && 
				r2_ly >= r1_ly && 
				r2_ry >= r1_ry  && 
				r2_ly <= r1_ry
				) { 
		  
		  int area_of_overlap; 
		  
		  
		  // 	//2 areas: lets fuirst assume the bigger portion juts out
		  // 	int area_of_overlap = r2->getWidth() * (r2_ry - r1_ry ); 
		  
		  // 	if( (double)area_of_overlap/areaOfR2 >= area_merge_ratio )
		  // 	  isNeighbor = ( true ); 
		  
		  // 	else { 
		  
		  area_of_overlap = r2->getWidth() * (r1_ry - r2_ly ); 
		  
		  if( area_of_overlap < 0 ) {
			perror(" 6: area of overlap error");
			//	    exit( 1 );
		  }
		  
		  
		  if( (double)area_of_overlap/areaOfR2 >= area_merge_ratio ) 
			isNeighbor = ( true ); 
		  
		  //	} 
		}
		
		//case 7: like csae 6 but at top
		else if( r2_lx >= r1_lx &&
				r2_ly <= r1_ly && 
				r2_rx <= r1_rx && 
				r2_ry <= r1_ry && 
				r2_ry >= r1_ly 
				) { 
		  
		  int area_of_overlap; 
		  
		  // 	int area_of_overlap = r2->getWidth() * (r1_ly - r2_ly ); 
		  
		  // 	  if( (double)area_of_overlap/areaOfR2 >= area_merge_ratio ) 
		  // 	    isNeighbor = ( true ); 
		  
		  // 	  else { 
		  
		  area_of_overlap = r2->getWidth() * (r2_ry - r1_ly );
		  
		  if( area_of_overlap < 0 ) {
			perror("7: area of overlap error ");
			//	      exit(1);
		  }
	      
		  
		  if( (double)area_of_overlap/areaOfR2 >= area_merge_ratio ) 
			isNeighbor = ( true ); 
		  //	  }
		}
		
		//case 8: the smaller box juts out of the left of the bigger box
		else if( r2_lx <= r1_lx && 
				r2_rx <= r1_rx && 
				r2_ly >= r1_ly && 
				r2_ry <= r1_ry && 
				r2_rx >= r1_lx ) { 
		  
		  int area_of_overlap;
		  
		  // 	int area_of_overlap = r2->getHeight() * ( r1_lx - r2_lx );
		  // 	if( (double)area_of_overlap/areaOfR2 >= area_merge_ratio ) 
		  // 	  isNeighbor = ( true ); 
		  
		  // 	else { 
		  
		  area_of_overlap = r2->getHeight() * (r2_rx - r1_lx); 
		  
		  if( area_of_overlap < 0 ) { 
			perror("8:area of overlap error");
			//	    exit(1);
		  }
		  
		  if( (double)area_of_overlap/areaOfR2 >= area_merge_ratio ) 
			isNeighbor = ( true ); 
		  //	}
		}
		
		//case 9: the smaller box is jutting out of the right
		else if( r2_rx >= r1_rx &&
				r2_lx >= r1_lx && 
				r2_ly >= r1_ly && 
				r2_ry <= r1_ry && 
				r1_rx >= r2_lx ) { 
		  
		  int area_of_overlap; 
		  
		  // 	int area_of_overlap = r2->getHeight() * (r2_rx - r1_rx );
		  // 	if( (double)area_of_overlap/areaOfR2 >= area_merge_ratio ) 
		  // 	  isNeighbor = ( true ); 
		  
		  // 	else{ 
		  
		  area_of_overlap = r2->getHeight() * ( r1_rx - r2_lx ) ; 
		  
		  if( area_of_overlap < 0 ) { 
			perror("9 : area of overlap error");
			//	    exit(1);
		  }
		  
		  if( (double)area_of_overlap/areaOfR2 >= area_merge_ratio ) 
			isNeighbor = ( true ); 
		  
		  //	}
		}
		
		
		if( isNeighbor && swapped ) 
		  temp.push_back( r1 );
		
		else if( isNeighbor && !swapped ) 
		  temp.push_back( r2 ); 
		
		// replace them back 
		if( swapped ) { 
		  MergeWindowClass *tempSwapper  = r2;
		  r2 = r1;
		  r1 = tempSwapper; 
		  tempSwapper = 0; 
		  swapped = false; 
		}
		
      }
      
	  
	  
	  
      /*
       * All neighbors are in temp. Do one of the following: 
       * 1. Pick the bigger one!!!! 
       *  
       */ 
      /*
       *
       * Picking the bigger one!!!!!!!!!!!!!!!!! might not lead to best results
       * 
       */ 
      
      /*
       * this one of the highest probablity one
       */      // double pmax = -2.0;       
	  //       int index = -1; 
      
	  //       for( int j = 0; j < temp.size() ; j++ ) { 
	  
	  // 	if( temp.at(j)->getFaceProbabilityResult() >= pmax ) { 
	  // 	  pmax = temp.at(j)->getFaceProbabilityResult(); 
	  // 	  index = j ; 
	  // 	}
	  
	  // 	temp.at(j)->setAsConsidered();  // In any case, all must be NOT considered again (considered flag should be set)
	  
	  //       }
      
      int areaMax = -1;
      int index = -1;
	  
      for( unsigned int j = 0 ; j < temp.size() ; j++ ) { 
		if( temp.at(j)->getHeight()*temp.at(j)->getWidth() >= areaMax ) { 
		  index = j ;
		  areaMax = temp.at(j)->getHeight() * temp.at(j)->getWidth(); 
		}
		temp.at(j)->setAsConsidered(); 
      } 
	  
	  
	  
      if( temp.size() != 0 ) 
		chosen.push_back( temp.at( index ) ); //this has the chosen one 
      
	  
      temp.clear(); 
	  
    }
	
    
    WindowClass* wc = 0 ;
    for( unsigned int i = 0 ; i < windows.size() ; i++ ) { 
      wc = windows.at( i ) ;
      delete wc; 
    }
    
    windows.clear();
    
	for( unsigned int i = 0; i < chosen.size(); i++ )  
      if( DISCARD_SMALL ) 
      { 
        if( chosen.at(i)->getHeight() > DISCARD_SMALL_DIMENSION && 
		   chosen.at(i)->getWidth() > DISCARD_SMALL_DIMENSION )
		  windows.push_back( new WindowClass( chosen.at(i)->getY(),
											 chosen.at(i)->getX(),
											 chosen.at(i)->getHeight(),
											 chosen.at(i)->getWidth(),
											 chosen.at(i)->getFaceProbabilityResult() ) );
	  }      
	  else
	  {
		windows.push_back( new WindowClass( chosen.at(i)->getY(),
										   chosen.at(i)->getX(),
										   chosen.at(i)->getHeight(),
										   chosen.at(i)->getWidth(),
										   chosen.at(i)->getFaceProbabilityResult() ) );					      
	  }
	
	
	chosen.clear(); 
	
	MergeWindowClass* mwc = 0 ;
	for( unsigned int i = 0 ; i < detections.size() ; i++ ) { 
	  mwc = detections.at( i ); 
	  delete mwc; 
	}
	detections.clear(); 
	
  }
  
  
  
  
  
  
  /** heuristic type 4 */ 
  void filterNoise4( ConfigClass& config, vector< WindowClass* >& windows) { 
    
    // Remove all the inner rectangles 
    int topLeftX = 0, topLeftY = 0, bottomRightX = 0, bottomRightY = 0;
    
    
    // the percentage of an overlapping window that is considered a part of another detection hit: 
    // i.e. merge if this much perctange of area overlaps.
    const double area = AREA_OF_SUBWINDOW_MERGE;
    
    vector< MergeWindowClass* > temp, final, detections; 
    
    
    /*
     * Might need to make some modifications here like the ones i did for mergeDetection
     * maybe include a getFaceProbabilityResult() based sorting and then carry out below
     * 
     * SORT getFaceProbabilityResult() 
     *
     */ 


    for( unsigned int i = 0; i < windows.size(); i++ )  
      detections.push_back( new MergeWindowClass( windows.at(i)->getY(), 
						  windows.at(i)->getX(),
						  windows.at(i)->getHeight(),
						  windows.at(i)->getWidth(),
						  windows.at(i)->getFaceProbabilityResult() ) );
    
    for( unsigned int i = 0; i < windows.size(); i++ ) { 
      
      MergeWindowClass* r1 = detections.at( i );
      
      if( !r1->isConsidered() ) { 
	
	topLeftX = r1->getX();
	topLeftY = r1->getY(); 
	bottomRightX = r1->getX() + r1->getWidth(); 
	bottomRightY = r1->getY() + r1->getHeight();
	
	
	for( unsigned int j = 0; j < windows.size(); j++ ) { 
	  
	  MergeWindowClass* r2 = detections.at( j );

	  int areaOfR2 = r2->getHeight() * r2->getWidth(); 
	  
	  // Case 1: The r2 is completely inside r1
	  if( j!=i && 
	      !r2->isConsidered() && 
	      r2->getX() >= topLeftX && 
	      r2->getX() + r2->getWidth() <= bottomRightX && 
	      r2->getY() >= topLeftY && 
	      r2->getY() + r2->getHeight() <= bottomRightY )  
	    r2->setAsConsidered(); 

	  // Case 2: the width of r2 is inside r1
	  else if( j!=i && 
		   !r2->isConsidered() && 
		   r2->getX() >= topLeftX && 
		   r2->getX() + r2->getWidth() <= bottomRightX ) { 
	    
	    // Two cases: The smaller rectangle is atleast 20% inside from top
	    if( r2->getY() + (int)( (1 - area) * r2->getHeight() ) >= topLeftY &&
		r2->getY() <= topLeftY && 
		r2->getY() + r2->getHeight() <= bottomRightY ) 
	      r2->setAsConsidered();

	    //Other case
	    if( r2->getY() + (int)( area * r2->getHeight() ) <= bottomRightY && 
		r2->getY() <= bottomRightY && 
		r2->getY() >= topLeftY )
	      r2->setAsConsidered(); 

	  } // else if(..

	  // Case 3: the height of r2 is inside r2
	  else if( j!=i && 
		   !r2->isConsidered() && 
		   r2->getY() >= topLeftY && 
		   r2->getY() + r2->getHeight() <= bottomRightY ) { 

	    if( r2->getX() + (int)( (1-area) * r2->getWidth() ) >= topLeftX && 
		r2->getX() <= topLeftX && 
		r2->getX() + r2->getWidth() <= bottomRightX ) 
	      r2->setAsConsidered(); 
	    
	    if( r2->getX() >= topLeftX && 
		r2->getX() + (int)( area * r2->getWidth() ) <= bottomRightX && 
		r2->getX() <= bottomRightX ) 
	      r2->setAsConsidered(); 
	  } // else if(..

	  
	  // Case 4: If just a portion is within the bigger rectanlge widthwise
	  else if( j!=i && 
		   !r2->isConsidered() && 
		   ( r2->getY() <= topLeftY && r2->getY() + r2->getHeight() <= bottomRightY ) ) { 

	    // Case a: the portion is jutting out of the right of the bigger rectange
	    if( r2->getX() >= topLeftX && r2->getX() + r2->getWidth() >= bottomRightX 
		&& r2->getX() <= bottomRightX ) { 
	      int areaOfOverlap = ( r2->getY() + r2->getHeight() - topLeftY ) * 
		( r2->getX() + r2->getWidth() - bottomRightX ); 
	      
	      double perctangeOverlap = (double)areaOfOverlap / (double)areaOfR2; 
	      
	      if( perctangeOverlap > area ) 
		r2->setAsConsidered(); 

	    } // Case a..

	    // Case b: the portion jutting out of the left of the bigger rectabgel 
	    else if( r2->getX() <= topLeftX && r2->getX() + r2->getWidth() >= topLeftX 
		     && r2->getX() + r2->getWidth() <= bottomRightX ) { 
	      
	      int areaOfOverlap = ( r2->getY() + r2->getHeight() - topLeftY) * 
		( r2->getX() + r2->getWidth() - topLeftX ); 

	      double perctangeOverlap = (double)areaOfOverlap / (double)areaOfR2; 
	      
	      if( perctangeOverlap > area ) 
		r2->setAsConsidered(); 
	    } // Case b .. 


	    // Case c: the portion is jutting out of left and right of the bigger rectangle 
	    else if( r2->getX() <= topLeftX && r2->getX() + r2->getWidth() >= bottomRightX ) { 

	      int areaOfOverlap = ( r2->getY() + r2->getHeight() - topLeftX ) * 
		( r2->getWidth() - bottomRightX  + topLeftX ); 

	      double perctangeOverlap = (double)areaOfOverlap / (double)areaOfR2; 
	      
	      if( perctangeOverlap > area )
		r2->setAsConsidered(); 

	    } // Case c..

	  } // Case 4.. 	  
	  
	} 
      }
    }
      
    
    WindowClass* wc = 0 ;
    for( unsigned int i = 0 ; i < windows.size() ; i++ ) { 
      wc = windows.at( i ) ;
      delete wc; 
    }
    
    windows.clear();
    
      for( unsigned int i = 0; i < detections.size(); i++ )  
	if( !detections.at(i)->isConsidered() ) 
	  windows.push_back( new WindowClass( detections.at(i)->getY(),
					      detections.at(i)->getX(),
					      detections.at(i)->getHeight(),
					      detections.at(i)->getWidth(),
					      detections.at(i)->getFaceProbabilityResult() ) );
	  
      

  
      
      MergeWindowClass* mwc = 0 ;
      for( unsigned int i = 0 ; i < detections.size() ; i++ ) { 
	mwc = detections.at( i ); 
	delete mwc; 
      }
      detections.clear(); 
      

    }
	

  
  /** heuristic type 5 */ 
  void filterNoise5( std::vector<UserDef::WindowClass*> &windows, ConfigClass& config , 
		     double scaleFactor , int moduleParam=1) { 
    
    
    int r1_lx, r1_ly, r1_rx, r1_ry, r2_lx, r2_ly, r2_rx, r2_ry; 
    r1_lx = r1_ly = r1_rx = r1_ry = r2_lx = r2_ly = r2_rx = r2_ry = 0; 
    
	const double area_merge_ratio = AREA_OF_SUBWINDOW_MERGE;
    
    
    vector< MergeWindowClass* > final, detections, chosen; 
    
  
    double face_probability[ windows.size() ]; 
	
	
    for( unsigned int i=0; i<windows.size(); ++i ) 
      face_probability[ i ] = windows.at(i)->getFaceProbabilityResult(); 
	
    std::sort( face_probability, face_probability + detections.size() ); 
	
    // temp place to store windows 
    vector< UserDef::WindowClass* > tempwindows;
    for( unsigned  int i = 0 ; i < windows.size() ; i++ ) 
      tempwindows.push_back( windows.at ( i ) ); 
    
    for(  int i= windows.size()-1; i!=-1; i-- ) { 
      
      for( unsigned int j=0; j<tempwindows.size() ; j++) 
		if( tempwindows.at(j) != 0 )
		  if( tempwindows.at(j)->getFaceProbabilityResult() == face_probability[i] ) { 
			
			detections.push_back( new MergeWindowClass( tempwindows.at(j)->getY(),
													   tempwindows.at(j)->getX(),
													   tempwindows.at(j)->getHeight(),
													   tempwindows.at(j)->getWidth(),
													   tempwindows.at(j)->getFaceProbabilityResult() ) );
			
			tempwindows.at(j) = 0 ; 
			break; 
			
		  }
      
    } 
    /*
     * End 'bad' sorting section : but seems to work
     */
    
    tempwindows.clear(); 
    
    
    // the code below is constructed with the assumption that all the rectangles are of the same size
    // it is mergedetections from the same scale after all
    
    vector< MergeWindowClass* > temp; 
    
    for( unsigned int i = 0; i < detections.size(); i++ ) { 
      
      if( detections.at(i)->isConsidered() ) 
		continue; 
      
      MergeWindowClass *r1 = detections.at(i); 
      
      /*
       * for each subwindow, I am going to consider all "neighbors", mark them all, and choose the best
       *  temp will store them. Put all 'to be considered as neighbors' in one set (temp). 
       */ 
      
      temp.push_back( r1 ); //: I am going to do this later on 
	  
	  
      bool isNeighbor = false; 
      
      for( unsigned int j = 0; j < detections.size(); j++ ) { 
		
		if( detections.at(j)->isConsidered() || j==i ) // either already considered or it is the same
		  continue; 
		
		MergeWindowClass *r2 = detections.at(j); //r2 is the other rectangle 
		
		bool swapped = false; 
		
		if( r2->getHeight() * r2->getWidth() > r1->getHeight() * r1->getWidth() )
		{ 
	      MergeWindowClass* tempRectangle = r1;
	      r1 = r2;
	      r2 = tempRectangle;
	      tempRectangle = 0; 
	      swapped = true; 
	    }
		
		r1_lx = r1->getX(); r1_ly = r1->getY();
		r1_rx = r1->getX() + r1->getWidth(); r1_ry = r1->getY() + r1->getHeight(); 
		
		r2_lx = r2->getX(); r2_ly = r2->getY(); 
		r2_rx = r2->getX() + r2->getWidth(); r2_ry = r2->getY() + r2->getHeight(); 
		
		
		
		int areaOfR2 = r2->getHeight() * r2->getWidth();
		//		int areaOfR1 = r1->getHeight() * r1->getWidth(); 
		
		// if r2 falls into any of teh below conditions, then it iwll be put into temp and the best one will be chosen
		
		// case1: r2 is topleft corner of r1
		if( r2_lx <= r1_lx && 
		   r2_ly <= r1_ly && 
		   r2_rx <= r1_rx && 
		   r2_ry <= r1_ry  && 
		   r2_rx >= r1_lx && 
		   r2_ry >= r1_ly 
		   ) { // is overlapping area passing threshold 
		  
		  int area_of_overlap; 
		  
		
		  area_of_overlap = (r2_rx - r1_lx) * (r2_ry - r1_ly); 
		 if( area_of_overlap < 0 ) { 
			perror("1: area of overlap < 0 "); 
			//	      exit( 1 ); 
		  }
		  
		  if( (double)area_of_overlap/areaOfR2 >=area_merge_ratio) 
			isNeighbor = true;
		  //	  }
		  
		} 
		
		// case 2: r2 is top right of r1
		else if( r2_lx >= r1_lx && 
				r2_ly <= r1_ly && 
				r2_rx >= r1_rx && 
				r2_ry <= r1_ry && 
				r2_lx <= r1_rx && 
				r2_ry >= r1_ly ) { 
		  
		  int area_of_overlap; 
		  
		  // 	  int area_of_overlap = (r1_ly - r2_ly) * r2->getWidth() + (r2_rx - r1_rx) * r2->getHeight()  - 
		  // 	    (r2_rx - r1_rx) * (r1_ly - r2_ly ); 
		  
		  
		  // 	  // if outer area
		  // 	  if( (double)area_of_overlap/areaOfR2 >= area_merge_ratio ) //consider this
		  // 	    isNeighbor = true;
		  
		  // 	  else { 
		  
		  area_of_overlap = (r2_ry - r1_ly) * ( r1_rx - r2_lx); 
		  
		  
		  if( area_of_overlap < 0 ) { 
			perror("2: area of overlap error");
			//	    exit( 1 );
		  }
		  
		  if( (double)area_of_overlap/areaOfR2 >= area_merge_ratio ) 
			isNeighbor  = true;
		  
		  // 	  }
		  
		  
		}
		
		
		// case 3: bottom left corner
		else if( r2_lx <= r1_lx && 
				r2_ly >= r1_ly && 
				r1_rx >= r2_rx && 
				r2_ry >= r1_ry && 
				r2_rx >= r1_lx && 
				r2_ly <= r2_ry ) { 
		  
		  int area_of_overlap = 0; 
		  
		  // 	  int area_of_overlap = r2->getHeight() * (r1_lx - r2_lx ) + 
		  // 	    r2->getWidth() * (r2_ry - r1_ry) - (r1_lx - r2_lx)*(r2_ry - r1_ry); 
		  
		  // 	  if( (double)area_of_overlap/areaOfR2 >= area_merge_ratio ) 
		  // 	    isNeighbor = ( true );
		  
		  // 	  else { 
		  area_of_overlap = (r2_rx - r1_lx) * ( r2_ry - r1_ly); 
		  
		  if( area_of_overlap < 0 ) { 
			perror("3: area of overlap error");
			//	      exit( 1 );
		  }
		  
		  if( (double)area_of_overlap/areaOfR2 >= area_merge_ratio ) 
			isNeighbor = ( true );
		  
		  //	  }
		  
		}
		
		//case 4": bottomr ight corner
		else if( r2_lx >= r1_lx && 
				r2_ly >= r1_ly && 
				r2_rx >= r1_rx && 
				r2_ry >= r1_ry && 
				r2_lx <= r1_rx && 
				r2_ly <= r1_ry
				) { 
		  
		  int area_of_overlap; 
		  
		  // 	  int area_of_overlap = r2->getHeight() * ( r2_rx - r1_rx ) +
		  // 	    r2->getWidth() * (r2_ry - r1_ry ) - 
		  // 	    (r2_rx - r1_rx ) * ( r2_ry - r1_ry); 
		  
		  // 	  if( (double)area_of_overlap/areaOfR2 >= area_merge_ratio ) 
		  // 	    isNeighbor = ( true );
		  
		  // 	  else { 
		  
		  area_of_overlap = (r1_rx - r2_lx ) * ( r1_ry - r2_ly ); 
		  if( area_of_overlap < 0 ) { 
			perror(" 4:area of overlap ");
			//	      exit( 1 ); 
		  }
		  
	      
		  
		  if( (double)area_of_overlap/areaOfR2 >= area_merge_ratio )
			isNeighbor = ( true );
		  //	  }
		  
		}
		
		
		//case 5: box is completely inside bigger box
		else if( r2_lx >= r1_lx && 
				r2_ly >= r1_ly && 
				r2_rx <= r1_rx && 
				r2_ry <= r1_ry ) // no need to check areas
		  isNeighbor = ( true ); 
		
		
		
		//case 6: the small rectangle is inside the region of the bigger one, but a small portion juts out ( that is, its not completely inside) [ at the bottom of the bigger rectange ] 
		else if( r2_lx >= r1_lx && 
				r2_rx <= r1_rx && 
				r2_ly >= r1_ly && 
				r2_ry >= r1_ry  && 
				r2_ly <= r1_ry
				) { 
		  
		  int area_of_overlap; 
		  
		  
		  // 	//2 areas: lets fuirst assume the bigger portion juts out
		  // 	int area_of_overlap = r2->getWidth() * (r2_ry - r1_ry ); 
		  
		  // 	if( (double)area_of_overlap/areaOfR2 >= area_merge_ratio )
		  // 	  isNeighbor = ( true ); 
		  
		  // 	else { 
		  
		  area_of_overlap = r2->getWidth() * (r1_ry - r2_ly ); 
		  
		  if( area_of_overlap < 0 ) {
			perror(" 6: area of overlap error");
			//	    exit( 1 );
		  }
		  
		  
		  if( (double)area_of_overlap/areaOfR2 >= area_merge_ratio ) 
			isNeighbor = ( true ); 
		  
		  //	} 
		}
		
		//case 7: like csae 6 but at top
		else if( r2_lx >= r1_lx &&
				r2_ly <= r1_ly && 
				r2_rx <= r1_rx && 
				r2_ry <= r1_ry && 
				r2_ry >= r1_ly 
				) { 
		  
		  int area_of_overlap; 
		  
		  // 	int area_of_overlap = r2->getWidth() * (r1_ly - r2_ly ); 
		  
		  // 	  if( (double)area_of_overlap/areaOfR2 >= area_merge_ratio ) 
		  // 	    isNeighbor = ( true ); 
		  
		  // 	  else { 
		  
		  area_of_overlap = r2->getWidth() * (r2_ry - r1_ly );
		  
		  if( area_of_overlap < 0 ) {
			perror("7: area of overlap error ");
			//	      exit(1);
		  }
	      
		  
		  if( (double)area_of_overlap/areaOfR2 >= area_merge_ratio ) 
			isNeighbor = ( true ); 
		  //	  }
		}
		
		//case 8: the smaller box juts out of the left of the bigger box
		else if( r2_lx <= r1_lx && 
				r2_rx <= r1_rx && 
				r2_ly >= r1_ly && 
				r2_ry <= r1_ry && 
				r2_rx >= r1_lx ) { 
		  
		  int area_of_overlap;
		  
		  // 	int area_of_overlap = r2->getHeight() * ( r1_lx - r2_lx );
		  // 	if( (double)area_of_overlap/areaOfR2 >= area_merge_ratio ) 
		  // 	  isNeighbor = ( true ); 
		  
		  // 	else { 
		  
		  area_of_overlap = r2->getHeight() * (r2_rx - r1_lx); 
		  
		  if( area_of_overlap < 0 ) { 
			perror("8:area of overlap error");
			//	    exit(1);
		  }
		  
		  if( (double)area_of_overlap/areaOfR2 >= area_merge_ratio ) 
			isNeighbor = ( true ); 
		  //	}
		}
		
		//case 9: the smaller box is jutting out of the right
		else if( r2_rx >= r1_rx &&
				r2_lx >= r1_lx && 
				r2_ly >= r1_ly && 
				r2_ry <= r1_ry && 
				r1_rx >= r2_lx ) { 
		  
		  int area_of_overlap; 
		  
		  // 	int area_of_overlap = r2->getHeight() * (r2_rx - r1_rx );
		  // 	if( (double)area_of_overlap/areaOfR2 >= area_merge_ratio ) 
		  // 	  isNeighbor = ( true ); 
		  
		  // 	else{ 
		  
		  area_of_overlap = r2->getHeight() * ( r1_rx - r2_lx ) ; 
		  
		  if( area_of_overlap < 0 ) { 
			perror("9 : area of overlap error");
			//	    exit(1);
		  }
		  
		  if( (double)area_of_overlap/areaOfR2 >= area_merge_ratio ) 
			isNeighbor = ( true ); 
		  
		  //	}
		}
		
		
		if( isNeighbor && swapped ) 
		  temp.push_back( r1 );
		
		else if( isNeighbor && !swapped ) 
		  temp.push_back( r2 ); 
		
		// replace them back 
		if( swapped ) 
		{ 
		  MergeWindowClass *tempSwapper  = r2;
		  r2 = r1;
		  r1 = tempSwapper; 
		  tempSwapper = 0; 
		  swapped = false; 
		}
		
      }
      
	  /*
	   * at this point all of the subwindows neighbors are collected. So, probably
	   * this part is pretty good since it has accurately collected all the neighbors.
	   * from here is the part of choosing the biggest one (filter3) or
	   * choosing the highest fmp valued one
	   * or choosing some kind of intersection
	   */
	  
      int areaMax = -1;
      int index = -1;
	  double fmp_average = 0.0; 

	  int module = moduleParam; // different heurisitcs for merging
	  
	  if( module == 0 ) 
	    { // this considers the two biggest rectangles and merges them 

	      //first find the biggest
	      for( unsigned int j = 0; j < temp.size(); j++ )
		{
		  if( temp.at(j)->getHeight() * temp.at(j)->getWidth() >= areaMax ) 
		    {
		      index = j;
		      areaMax = temp.at(j)->getHeight() * temp.at(j)->getWidth(); 
		    }
		  temp.at(j)->setAsConsidered(); 
		}
	      int _areaMax = -1; 
	      int _index = -1; 
	      int considerSecond = false; 
	      if( temp.size()!=0 && temp.size()!=1 )
		{
		  considerSecond = true;
		  for( int j = 0; j < (int)temp.size(); j++ )
		    {
		      if( j!= index )
			{
			  if( temp.at(j)->getHeight() * temp.at(j)->getWidth() >= _areaMax ) 
			    {
			      _index = j;
			      _areaMax = temp.at(j)->getHeight() * temp.at(j)->getWidth(); 
			    }

			}
		
		    }

		}

	      if( !considerSecond ) 
		{
		  chosen.push_back( new MergeWindowClass( temp.at(index)->getY(), 
							  temp.at(index)->getX(), 
							  temp.at(index)->getHeight(), 
							  temp.at(index)->getWidth(), 
							  temp.at(index)->getFaceProbabilityResult() 
							  ) ); 
		}

	      else 
		{
		  
		  printf("the values are: %d %d %d %d %f\n", temp.at(index)->getY(), temp.at(index)->getX(), temp.at(index)->getHeight(), temp.at(index)->getWidth(), 
			 temp.at(index)->getFaceProbabilityResult() ); 
		  printf(" and %d %d %d %d %f\n", temp.at(_index)->getY() , temp.at(_index)->getX(), temp.at(_index)->getHeight(), temp.at(_index)->getWidth(), 
			 temp.at(_index)->getFaceProbabilityResult() ); 


		  chosen.push_back( new MergeWindowClass( (int)(0.5* (temp.at(index)->getY() + temp.at(_index)->getY() ) ), 
							  (int)(0.5 * (temp.at(index)->getY() + temp.at(_index)->getY() ) ),
							  (int)(0.5 * (temp.at(index)->getHeight() + temp.at(_index)->getHeight() ) ),
							  (int)(0.5 * (temp.at(index)->getWidth() + temp.at(_index)->getWidth() ) ),
							  temp.at(index)->getFaceProbabilityResult()>temp.at(_index)->getFaceProbabilityResult()?temp.at(index)->getFaceProbabilityResult(): temp.at(_index)->getFaceProbabilityResult() ) ); 
		
		}
	      
	      
	      
	    }
	  
	  
	  if( module == 1 ) // this is the filterNoise3 module
	  {
		
		for( unsigned int j = 0 ; j < temp.size() ; j++ ) { 
		  if( temp.at(j)->getHeight()*temp.at(j)->getWidth() >= areaMax ) { 
			index = j ;
			areaMax = temp.at(j)->getHeight() * temp.at(j)->getWidth(); 
		  }
		  temp.at(j)->setAsConsidered(); 
		}
		
		if( temp.size() != 0 )
		  chosen.push_back( new MergeWindowClass( temp.at(index)->getY(), 
							  temp.at(index)->getX(),
							  temp.at(index)->getHeight(), 
							  temp.at(index)->getWidth(), 
							  temp.at(index)->getFaceProbabilityResult() ) ) ;
		//temp.at( index ) ); //this has the chosen one 
	  }	  
	  else if( module == 2 ) // this is the averaging technique
	  {
		/* this method works by taking the average of the left corner coordinates (x,y)
		 and the average of the heights and weights and using them to draw
		 one single rectangle. Does not work well. Probably because there are so many tiny 
		 rectangels that the average is just one single tiny rectangle 
		 */
		int lx_average = 0, ly_average = 0, w_average = 0, h_average = 0;
		int number = -1;
		
		if( temp.size() == 0 ) 
		  number = 1;
		else 
		  number = temp.size(); 
		
		for( unsigned int j = 0; j < temp.size(); j++ ) 
		{
		  lx_average += temp.at(j)->getX(); 
		  ly_average += temp.at(j)->getY(); 
		  w_average += temp.at(j)->getWidth();
		  h_average += temp.at(j)->getHeight(); 
		  fmp_average += temp.at(j)->getFaceProbabilityResult(); 
		  temp.at(j)->setAsConsidered(); 
		}
		
		
		lx_average = (int)( lx_average/number );
		ly_average = (int)( ly_average/number );
		w_average = (int)( w_average/number );
		h_average = (int)( h_average/number ); 
		fmp_average /= (double)( number * 1.0 ); 
		
				
		if( temp.size() != 0 ) 
		  chosen.push_back( new MergeWindowClass( 
												 ly_average, 
												 lx_average, 
												 h_average, 
												 w_average,
												 fmp_average ) );
		
	  }

	  else if( module == 3 ) 
	  {
		/*
		 * this heurisitic considers a weighted combination. 
		 * it first takes the biggest rectangle (like filter 3 which worked so well)
		 * and it takes the smallest one. 
		 * it then considers averaging by ( r1*2 + biggest + smallest )/4
		 */
		int lx_average = 0, ly_average = 0, h_average = 0, w_average = 0; 
		lx_average = 2 * r1->getX();
		ly_average = 2 * r1->getY(); 
		h_average = 2 * r1->getHeight(); 
		w_average = 2 * r1->getWidth(); 
		r1->setAsConsidered();
		
		MergeWindowClass *biggest, *smallest; 
		int area_big_comp = -1; 
		int area_small_comp = 1000; 
		
		for( unsigned int i = 1; i < temp.size(); i++ )
		{  
		  if( temp.at(i)->getHeight() * temp.at(i)->getWidth() > area_big_comp )
		  {
			biggest = temp.at(i); 
			area_big_comp = biggest->getHeight() * biggest->getWidth();
		  }
		 
		  if( temp.at(i)->getHeight() * temp.at(i)->getWidth() < area_small_comp )
		  {
			smallest = temp.at(i);
			area_small_comp = smallest->getHeight() * smallest->getWidth();
		  }
		  
		  temp.at(i)->setAsConsidered(); 
		  
		}
		
		lx_average = biggest->getX() + smallest->getX();
		ly_average = biggest->getY() + smallest->getY(); 
		h_average = biggest->getHeight() + smallest->getHeight(); 
		w_average = biggest->getWidth() + smallest->getWidth(); 
		
		lx_average = (int)(lx_average/4); 
		ly_average = (int)(ly_average/4);
		h_average = (int)(h_average/4);
		w_average = (int)(w_average/4);
		
		chosen.push_back( new MergeWindowClass( 
											   ly_average,
											   lx_average,
											   h_average,
											   w_average,
											   r1->getFaceProbabilityResult() 
											   ) );
											   				
	  }
	  temp.clear(); 
	  
    }
	
    
    WindowClass* wc = 0 ;
    for( unsigned int i = 0 ; i < windows.size() ; i++ ) { 
      wc = windows.at( i ) ;
      delete wc; 
    }
    
    windows.clear();
    
	for( unsigned int i = 0; i < chosen.size(); i++ )  
	{
	  

      if( DISCARD_SMALL ) 
      { 
        if( chosen.at(i)->getHeight() > DISCARD_SMALL_DIMENSION && 
		   chosen.at(i)->getWidth() > DISCARD_SMALL_DIMENSION )
		  windows.push_back( new WindowClass( chosen.at(i)->getY(),
											 chosen.at(i)->getX(),
											 chosen.at(i)->getHeight(),
											 chosen.at(i)->getWidth(),
											 chosen.at(i)->getFaceProbabilityResult() ) );
	  }      
	  else
	  {
		windows.push_back( new WindowClass( chosen.at(i)->getY(),
										   chosen.at(i)->getX(),
										   chosen.at(i)->getHeight(),
										   chosen.at(i)->getWidth(),
										   chosen.at(i)->getFaceProbabilityResult() ) );					      
	  }
	
	}	
	
	MergeWindowClass* mwc = 0 ;
	for( unsigned int i = 0; i < chosen.size(); i++ ) 
	{
	  mwc = chosen.at(i);
	  delete mwc; 
	}
	chosen.clear(); 
	
	for( unsigned int i = 0; i < detections.size(); i++ )
	{ 
	  mwc = detections.at(i); 
	  delete mwc; 
	}
	detections.clear(); 
	
  }
  

  /** heuristic type 6 */ 
  void filterNoise6( std::vector<UserDef::WindowClass*> &windows, ConfigClass& config) 
  {
 
    int r1_lx, r1_ly, r1_rx, r1_ry, r2_lx, r2_ly, r2_rx, r2_ry; 
    r1_lx = r1_ly = r1_rx = r1_ry = r2_lx = r2_ly = r2_rx = r2_ry = 0; 
    
    /*
     * the percentage of an overlapping window that is considered a part of another detection hit: 
     * i.e. merge if this much perctange of area overlaps.
     */ const double area_merge_ratio = AREA_OF_SUBWINDOW_MERGE;
    
    
    vector< MergeWindowClass* > final, detections, chosen; 
    
    /* 
     * Part of the old code 
     */    //     for( int i = 0; i < windows.size(); i++ )  
	//       detections.push_back( new MergeWindowClass( windows.at(i)->getY(), 
	// 						  windows.at(i)->getX(),
	// 						  windows.at(i)->getHeight(),
	// 						  windows.at(i)->getWidth(),
	// 						  windows.at(i)->getFaceProbabilityResult() ) );
	
	
	
    /*
     * *******************
     * NEED TO SORT the probabilities so that they are considere from hightest probability to lowest probability
     * due to the price of merging procedures
     * TODO!!!! 
     ********************
     */ 
    
	
    /* 
     * Begin 'bad' sort section 
     */    
    double face_probability[ windows.size() ]; 
	
	
    for( unsigned int i=0; i<windows.size(); ++i ) 
      face_probability[ i ] = windows.at(i)->getFaceProbabilityResult(); 
	
    std::sort( face_probability, face_probability + detections.size() ); 
	
    // temp place to store windows 
    vector< UserDef::WindowClass* > tempwindows;
    for( unsigned int i = 0 ; i < windows.size() ; i++ ) 
      tempwindows.push_back( windows.at ( i ) ); 
    
    for(  int i= windows.size()-1; i!=-1; i-- ) { 
      
      for( unsigned int j=0; j<tempwindows.size() ; j++) 
		if( tempwindows.at(j) != 0 )
		  if( tempwindows.at(j)->getFaceProbabilityResult() == face_probability[i] ) { 
			
			detections.push_back( new MergeWindowClass( tempwindows.at(j)->getY(),
													   tempwindows.at(j)->getX(),
													   tempwindows.at(j)->getHeight(),
													   tempwindows.at(j)->getWidth(),
													   tempwindows.at(j)->getFaceProbabilityResult() ) );
			
			tempwindows.at(j) = 0 ; 
			break; 
			
		  }
      
    } 
    /*
     * End 'bad' sorting section : but seems to work
     */
    
    tempwindows.clear(); 
    
    
    // the code below is constructed with the assumption that all the rectangles are of the same size
    // it is mergedetections from the same scale after all
    
    vector< MergeWindowClass* > temp; 
    
    for( unsigned int i = 0; i < detections.size(); i++ ) { 
      
      if( detections.at(i)->isConsidered() ) 
		continue; 
      
      MergeWindowClass *r1 = detections.at(i); 
      
      /*
       * for each subwindow, I am going to consider all "neighbors", mark them all, and choose the best
       *  temp will store them. Put all 'to be considered as neighbors' in one set (temp). 
       */ 
      
      temp.push_back( r1 ); //: I am going to do this later on 
	  
	  
      bool isNeighbor = false; 
      
      for( unsigned int j = 0; j < detections.size(); j++ ) { 
		
		if( detections.at(j)->isConsidered() || j==i ) // either already considered or it is the same
		  continue; 
		
		MergeWindowClass *r2 = detections.at(j); //r2 is the other rectangle 
		
		bool swapped = false; 
		
		/*
		 * ********* 
		 * Since it can happen that the smaller rectangle is r1, I am going to try and see if I can swtich them
		 * i.e. the bigger rectanlge is always r1 : so a simple swap
		 */
		
		if( r2->getHeight() * r2->getWidth() > r1->getHeight() * r1->getWidth() ){ 
	      MergeWindowClass* tempRectangle = r1;
	      r1 = r2;
	      r2 = tempRectangle;
	      tempRectangle = 0; 
	      swapped = true; 
	    }
		
		r1_lx = r1->getX(); r1_ly = r1->getY();
		r1_rx = r1->getX() + r1->getWidth(); r1_ry = r1->getY() + r1->getHeight(); 
		
		r2_lx = r2->getX(); r2_ly = r2->getY(); 
		r2_rx = r2->getX() + r2->getWidth(); r2_ry = r2->getY() + r2->getHeight(); 
		
		
		
		int areaOfR2 = r2->getHeight() * r2->getWidth();
		//		int areaOfR1 = r1->getHeight() * r1->getWidth(); 
		
		// if r2 falls into any of teh below conditions, then it iwll be put into temp and the best one will be chosen
		
		// case1: r2 is topleft corner of r1
		if( r2_lx <= r1_lx && 
		   r2_ly <= r1_ly && 
		   r2_rx <= r1_rx && 
		   r2_ry <= r1_ry  && 
		   r2_rx >= r1_lx && 
		   r2_ry >= r1_ly 
		   ) { // is overlapping area passing threshold 
		  
		  int area_of_overlap; 
		  
		  // I seem to have wrongly reasoned that there are 2 cases: but there is only one isn't there?
		  
		  // 	  // maximum portion is outside : get outside area
		  // 	  int area_of_overlap = (r1_lx - r2_lx) * r2->getHeight() + (r1_ly - r2_ly) * r2->getWidth() 
		  // 	    - (r1_lx - r2_lx) * (r1_ly - r2_ly); 
		  
		  // 	  area_of_overlap = areaOfR2 - area_of_overlap; 
		  
		  // 	  if( (double)area_of_overlap/areaOfR2 >= area_merge_ratio ) // consider this
		  // 	    isNeighbor = true; 
		  
		  // 	  else { 
		  area_of_overlap = (r2_rx - r1_lx) * (r2_ry - r1_ly); 
		  
		  // simple check : redundant
		  if( area_of_overlap < 0 ) { 
			perror("1: area of overlap < 0 "); 
			//	      exit( 1 ); 
		  }
		  
		  if( (double)area_of_overlap/areaOfR2 >=area_merge_ratio) 
			isNeighbor = true;
		  //	  }
		  
		} 
		
		// case 2: r2 is top right of r1
		else if( r2_lx >= r1_lx && 
				r2_ly <= r1_ly && 
				r2_rx >= r1_rx && 
				r2_ry <= r1_ry && 
				r2_lx <= r1_rx && 
				r2_ry >= r1_ly ) { 
		  
		  int area_of_overlap; 
		  
		  // 	  int area_of_overlap = (r1_ly - r2_ly) * r2->getWidth() + (r2_rx - r1_rx) * r2->getHeight()  - 
		  // 	    (r2_rx - r1_rx) * (r1_ly - r2_ly ); 
		  
		  
		  // 	  // if outer area
		  // 	  if( (double)area_of_overlap/areaOfR2 >= area_merge_ratio ) //consider this
		  // 	    isNeighbor = true;
		  
		  // 	  else { 
		  
		  area_of_overlap = (r2_ry - r1_ly) * ( r1_rx - r2_lx); 
		  
		  
		  if( area_of_overlap < 0 ) { 
			perror("2: area of overlap error");
			//	    exit( 1 );
		  }
		  
		  if( (double)area_of_overlap/areaOfR2 >= area_merge_ratio ) 
			isNeighbor  = true;
		  
		  // 	  }
		  
		  
		}
		
		
		// case 3: bottom left corner
		else if( r2_lx <= r1_lx && 
				r2_ly >= r1_ly && 
				r1_rx >= r2_rx && 
				r2_ry >= r1_ry && 
				r2_rx >= r1_lx && 
				r2_ly <= r2_ry ) { 
		  
		  int area_of_overlap = 0; 
		  
		  // 	  int area_of_overlap = r2->getHeight() * (r1_lx - r2_lx ) + 
		  // 	    r2->getWidth() * (r2_ry - r1_ry) - (r1_lx - r2_lx)*(r2_ry - r1_ry); 
		  
		  // 	  if( (double)area_of_overlap/areaOfR2 >= area_merge_ratio ) 
		  // 	    isNeighbor = ( true );
		  
		  // 	  else { 
		  area_of_overlap = (r2_rx - r1_lx) * ( r2_ry - r1_ly); 
		  
		  if( area_of_overlap < 0 ) { 
			perror("3: area of overlap error");
			//	      exit( 1 );
		  }
		  
		  if( (double)area_of_overlap/areaOfR2 >= area_merge_ratio ) 
			isNeighbor = ( true );
		  
		  //	  }
		  
		}
		
		//case 4": bottomr ight corner
		else if( r2_lx >= r1_lx && 
				r2_ly >= r1_ly && 
				r2_rx >= r1_rx && 
				r2_ry >= r1_ry && 
				r2_lx <= r1_rx && 
				r2_ly <= r1_ry
				) { 
		  
		  int area_of_overlap; 
		  
		  // 	  int area_of_overlap = r2->getHeight() * ( r2_rx - r1_rx ) +
		  // 	    r2->getWidth() * (r2_ry - r1_ry ) - 
		  // 	    (r2_rx - r1_rx ) * ( r2_ry - r1_ry); 
		  
		  // 	  if( (double)area_of_overlap/areaOfR2 >= area_merge_ratio ) 
		  // 	    isNeighbor = ( true );
		  
		  // 	  else { 
		  
		  area_of_overlap = (r1_rx - r2_lx ) * ( r1_ry - r2_ly ); 
		  if( area_of_overlap < 0 ) { 
			perror(" 4:area of overlap ");
			//	      exit( 1 ); 
		  }
		  
	      
		  
		  if( (double)area_of_overlap/areaOfR2 >= area_merge_ratio )
			isNeighbor = ( true );
		  //	  }
		  
		}
		
		
		//case 5: box is completely inside bigger box
		else if( r2_lx >= r1_lx && 
				r2_ly >= r1_ly && 
				r2_rx <= r1_rx && 
				r2_ry <= r1_ry ) // no need to check areas
		  isNeighbor = ( true ); 
		
		
		
		//case 6: the small rectangle is inside the region of the bigger one, but a small portion juts out ( that is, its not completely inside) [ at the bottom of the bigger rectange ] 
		else if( r2_lx >= r1_lx && 
				r2_rx <= r1_rx && 
				r2_ly >= r1_ly && 
				r2_ry >= r1_ry  && 
				r2_ly <= r1_ry
				) { 
		  
		  int area_of_overlap; 
		  
		  
		  // 	//2 areas: lets fuirst assume the bigger portion juts out
		  // 	int area_of_overlap = r2->getWidth() * (r2_ry - r1_ry ); 
		  
		  // 	if( (double)area_of_overlap/areaOfR2 >= area_merge_ratio )
		  // 	  isNeighbor = ( true ); 
		  
		  // 	else { 
		  
		  area_of_overlap = r2->getWidth() * (r1_ry - r2_ly ); 
		  
		  if( area_of_overlap < 0 ) {
			perror(" 6: area of overlap error");
			//	    exit( 1 );
		  }
		  
		  
		  if( (double)area_of_overlap/areaOfR2 >= area_merge_ratio ) 
			isNeighbor = ( true ); 
		  
		  //	} 
		}
		
		//case 7: like csae 6 but at top
		else if( r2_lx >= r1_lx &&
				r2_ly <= r1_ly && 
				r2_rx <= r1_rx && 
				r2_ry <= r1_ry && 
				r2_ry >= r1_ly 
				) { 
		  
		  int area_of_overlap; 
		  
		  // 	int area_of_overlap = r2->getWidth() * (r1_ly - r2_ly ); 
		  
		  // 	  if( (double)area_of_overlap/areaOfR2 >= area_merge_ratio ) 
		  // 	    isNeighbor = ( true ); 
		  
		  // 	  else { 
		  
		  area_of_overlap = r2->getWidth() * (r2_ry - r1_ly );
		  
		  if( area_of_overlap < 0 ) {
			perror("7: area of overlap error ");
			//	      exit(1);
		  }
	      
		  
		  if( (double)area_of_overlap/areaOfR2 >= area_merge_ratio ) 
			isNeighbor = ( true ); 
		  //	  }
		}
		
		//case 8: the smaller box juts out of the left of the bigger box
		else if( r2_lx <= r1_lx && 
				r2_rx <= r1_rx && 
				r2_ly >= r1_ly && 
				r2_ry <= r1_ry && 
				r2_rx >= r1_lx ) { 
		  
		  int area_of_overlap;
		  
		  // 	int area_of_overlap = r2->getHeight() * ( r1_lx - r2_lx );
		  // 	if( (double)area_of_overlap/areaOfR2 >= area_merge_ratio ) 
		  // 	  isNeighbor = ( true ); 
		  
		  // 	else { 
		  
		  area_of_overlap = r2->getHeight() * (r2_rx - r1_lx); 
		  
		  if( area_of_overlap < 0 ) { 
			perror("8:area of overlap error");
			//	    exit(1);
		  }
		  
		  if( (double)area_of_overlap/areaOfR2 >= area_merge_ratio ) 
			isNeighbor = ( true ); 
		  //	}
		}
		
		//case 9: the smaller box is jutting out of the right
		else if( r2_rx >= r1_rx &&
				r2_lx >= r1_lx && 
				r2_ly >= r1_ly && 
				r2_ry <= r1_ry && 
				r1_rx >= r2_lx ) { 
		  
		  int area_of_overlap; 
		  
		  // 	int area_of_overlap = r2->getHeight() * (r2_rx - r1_rx );
		  // 	if( (double)area_of_overlap/areaOfR2 >= area_merge_ratio ) 
		  // 	  isNeighbor = ( true ); 
		  
		  // 	else{ 
		  
		  area_of_overlap = r2->getHeight() * ( r1_rx - r2_lx ) ; 
		  
		  if( area_of_overlap < 0 ) { 
			perror("9 : area of overlap error");
			//	    exit(1);
		  }
		  
		  if( (double)area_of_overlap/areaOfR2 >= area_merge_ratio ) 
			isNeighbor = ( true ); 
		  
		  //	}
		}
		
		
		if( isNeighbor && swapped ) 
		  temp.push_back( r1 );
		
		else if( isNeighbor && !swapped ) 
		  temp.push_back( r2 ); 
		
		// replace them back 
		if( swapped ) { 
		  MergeWindowClass *tempSwapper  = r2;
		  r2 = r1;
		  r1 = tempSwapper; 
		  tempSwapper = 0; 
		  swapped = false; 
		}
		
      }
      
	  
	  
	  
      /*
       * All neighbors are in temp. Do one of the following: 
       * 1. Pick the bigger one!!!! 
       *  
       */ 
      /*
       *
       * Picking the bigger one!!!!!!!!!!!!!!!!! might not lead to best results
       * 
       */ 
      
      /*
       * this one of the highest probablity one
       */      // double pmax = -2.0;       
	  //       int index = -1; 
      
	  //       for( int j = 0; j < temp.size() ; j++ ) { 
	  
	  // 	if( temp.at(j)->getFaceProbabilityResult() >= pmax ) { 
	  // 	  pmax = temp.at(j)->getFaceProbabilityResult(); 
	  // 	  index = j ; 
	  // 	}
	  
	  // 	temp.at(j)->setAsConsidered();  // In any case, all must be NOT considered again (considered flag should be set)
	  
	  //       }
      
      double probMax = 0.0;

      int index = -1;
	  
      for( unsigned int j = 0 ; j < temp.size() ; j++ ) { 
	if( temp.at(j)->getFaceProbabilityResult() >= probMax ) 
	  {
	    index = j;
	    probMax = temp.at(j)->getFaceProbabilityResult(); 
	  }
		// if( temp.at(j)->getHeight()*temp.at(j)->getWidth() >= areaMax ) { 
// 		  index = j ;
// 		  areaMax = temp.at(j)->getHeight() * temp.at(j)->getWidth(); 
// 		}
		temp.at(j)->setAsConsidered(); 
      } 
	  
	  
	  
      if( temp.size() != 0 ) 
		chosen.push_back( temp.at( index ) ); //this has the chosen one 
      
	  
      temp.clear(); 
	  
    }
	
    
    WindowClass* wc = 0 ;
    for( unsigned int i = 0 ; i < windows.size() ; i++ ) { 
      wc = windows.at( i ) ;
      delete wc; 
    }
    
    windows.clear();
    
    for( unsigned int i = 0; i < chosen.size(); i++ )  
   
      windows.push_back( new WindowClass( chosen.at(i)->getY(),
					  chosen.at(i)->getX(),
					  chosen.at(i)->getHeight(),
					  chosen.at(i)->getWidth(),
					  chosen.at(i)->getFaceProbabilityResult() ) );					      
	
	
    chosen.clear(); 
	
    MergeWindowClass* mwc = 0 ;
    for( unsigned int i = 0 ; i < detections.size() ; i++ ) { 
	  mwc = detections.at( i ); 
	  delete mwc; 
	}
	detections.clear();

    
    

  }
  


}

