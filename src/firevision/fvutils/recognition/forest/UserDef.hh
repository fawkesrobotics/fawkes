#include "Parameters.hh"
#include "CommonHeaders.hh"
#include <opencv/cv.h>


#ifndef __userdef__hh__
#define __userdef__hh__

/***************************************************************************
 *  UserDef.hh - Header file for object recognition with random forests: user specific modules, specific to our framework 
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

using namespace std;

namespace UserDef {
  
  
  /**
   * Height and Width information for an Image
   * just a wrapper class
   */
  class ImageInfoClass : public CvSize {
    
  public:
    ImageInfoClass(){
      height = -1;
      width = -1;
    }
    /** set height information 
     * \param h integer denoting height
     */ 
    void setHeight(int h){height=h;}
    /** set width information 
     * \param w integer denoting width
     */ 
    void setWidth(int w){width = w;}
    /** get the height set */
    int getHeight() const { return height;}
    /** get the width considered */ 
    int getWidth() const {return width;}
  };
  
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
    vector< int* > iiVector; 
    
    /** returns the size of the vector */ 
    int size() const { return iiVector.size(); } 
    /** clears the items of the vector */ 
    void clear() { iiVector.clear(); } 

    /** sets the height of each image in the vector - the images are assumed to be scaled to the same size 
     * \param height set the height to this integer 
     */ 
    void setHeight(int height) { heightOfEachImage = height; }
    /** set the width of each image in the vector  - the images are assumed to be scaled to the same size 
     * \param width set the width to this integer value */ 
    void setWidth(int width) { widthOfEachImage = width; }
    /** returns the height of an image - the images are assumed to be scaled to the same size */
    int getHeight() { return heightOfEachImage; }
    /** returns the width of an image - the images are assumed to be scaled to the same size */ 
    int getWidth() { return widthOfEachImage; }

  };


  /** For a given file location, return that image in the IplImage* format */ 
  IplImage* getImageFromLocation(const string);
  
  
  /** compare 2 integrla images */ 
  bool compareIntegralImages( int*, int*, int, int ); 

  /** For a given image, calculate its integralimage */
  void calculateIntegralImage(IplImage*, int*);

  /** get the rectangular feature score */ 
  double getRectangularIntegralImagefeature(int* integralImageCalculated, int xMain, int yMain, int x, int y, int height, int width, int haarFeature);
  
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
   * Quick access of BW pixel intensities
   */
  typedef struct { 
    /** the pixel intensity */ 
    unsigned char r;
  } BwPixel;

  typedef Image<BwPixel> BwImage;

  /** ImageGroup - it is a vector of IplImage pointers */
  typedef vector<IplImage*> ImageGroup;

  // Redundant data structure
//   class ImageGroup {
//   public: 
//     vector<IplImage* > imageGroup;
//     ImageGroup(){}
//     inline int size() const { return imageGroup.size(); }
//     inline void push_back(IplImage* image=0) { imageGroup.push_back(image); }
//     inline IplImage* back() const { return imageGroup.back();}
//     inline void pop_back(){ imageGroup.pop_back();}
//     inline IplImage* at(int i) const { return imageGroup.at(i); }
    
//     ~ImageGroup() { }
//   };
  
  /**  
   * Window Class - stores the details of the window detected
   * parent class of MergeWindowClass 
   */
  class WindowClass: public CvRect { 

  private:
    /** fmp value */ 
    double faceProbabilityResult;

  public:
    /** get the Y coordinate */ 
    int getY() const { return y; }
    /** get the X coordinate */
    int getX() const { return x; }
    /** get the height of the random rectangle */
    int getHeight() const { return height; }
    /** get the width of the random rectangle */
    int getWidth() const { return width; }
    /** get the FMp values */ 
    double getFaceProbabilityResult() const { return faceProbabilityResult; }
    
    /** constrcutor */ 
    WindowClass() { 
      y = x = height = width = 0; 
      faceProbabilityResult = 0.0; 
    }
    /** construcrtor from another WindowClass Instance 
     * \param wc the window class to copy from */ 
    WindowClass(WindowClass& wc) {       
      y = wc.y;
      x = wc.x;
      height = wc.height;
      width = wc.width;
      faceProbabilityResult = wc.getFaceProbabilityResult(); 
    }

    /** build window class from top left coordinates 
     * \param Y the y coordinate 
     * \param X the x coordinate */
    WindowClass(int Y, int X) { y = Y; x = X; 
      height = width = 0; 
      faceProbabilityResult = 0.0; 
    }
    
    /** build window class from height, width, x and y of a random rectangle 
     * \param Y the Y coordindate 
     * \param X the X coordinate 
     * \param Height the height of the detection window 
     * \param Width the width of the detection window 
     * \param FaceProbabilityResult the fmp value of the detection window */ 
    WindowClass(int Y, int X, int Height, int Width, double FaceProbabilityResult) { 
      
      y = Y;
      x = X; 
      height = Height; 
      width = Width; 
      faceProbabilityResult = FaceProbabilityResult; 
    }
  };


    
  /** 
   * Config class stores all details necessary as to where 
   * the files are located or the offline learnt tree is located.
   * the number of classes, the forest size 
   */
  class ConfigClass {
  private:
    /** the training images location */ 
    string trainImages_;
    /** the test images location */ 
    string testImages_;
    /** the global fmp  value*/ 
    int detection_threshold; 
    /** detection heuristic */ 
    int subwindow_range;
    /** default location of the training images */ 
    string trainIntegralImagesLoc_;
    /** default location of the test images */ 
    string testIntegralImagesLoc_;
    /** fmp value global threshold */ 
    double face_map_threshold; 
    /** number of the identities */ 
	int nclasses; 
    /** offline or online training */
    bool relearnStatusValue_;
    

  public:
    /** Given the sliding window implementation, imageInfoInstance refers to size that the tree was trained with (32X32 and so on). */
    ImageInfoClass imageInfoInstance;
    /** testImageInfoInstance details the test Image submitted. Essentially, we look for faces of the size we trained the tree with(32X32) which are the dimensions of the window as well in the whole test image. */
    ImageInfoClass testImageInfoInstance;
    /** a temporary storgage for the detection windows */ 
    vector<WindowClass*> windows;
    /** global collection of detection windows */ 
    vector<WindowClass*> forestWindows;
    /** limitations of the feature size */ 
    int rectangular_feature_size;
    /** file to store log messages */ 
    ofstream logfile;
    /** file to store all detections */
    ofstream detectionsFile;
    /** file to store fmp values */ 
    ofstream faceMapFile;
    /** the trainig images */ 
    ImageGroup* images;
    /** file to store all the detection results */ 
    ofstream globalDetectionsFile;
    /** image grop of the test images */ 
    ImageGroup* testImages;
    /** histogram for the entire forest  */ 
    int nodes_nos_faces; // nodes_nos_faces, nodes_nos_nonfaces count the number of faces in all the nodes all the path
    /** histogram for the entire forest */
    int nodes_nos_nonfaces; 

//     class { 

//       // Anon class created because I would like to store the actual integr arrays (integral Images)
//       // Whilst, I would like to store the address of these integer arrays in VectorOfIntegralImage class

//     private: 
//       int heightOfEachImage; 
//       int widthOfEachImage;

//     public: 
    
//       vector< int* > iiVector;
      
//       int size() const { return iiVector.size(); } 
//       void clear() { iiVector.clear(); }

//       void setHeight( int h ) { heightOfEachImage = h; }
//       void setWidth( int w ) { widthOfEachImage = w; } 
//       int getHeight() const { return heightOfEachImage; } 
//       int getWidth() const { return widthOfEachImage; }

//     } integralImages[NCLASSES], testIntegralImages[NCLASSES]; 

    
    /** the integral images of the training samples */ 
    VectorOfIntegralImages *integralImages; 
    /** the integral images of the test sampels */ 
    VectorOfIntegralImages *testIntegralImages; 

    /** maintaining the global node index */ 
    int globalNodeIndex;
    
    /** constrcutor from the number of object classes/identities 
     * \param number_of_classes the number of identities */
    ConfigClass(int number_of_classes){
	  
	  
      rectangular_feature_size = RECTANGULAR_FEATURE_SIZE;
//       images = new ImageGroup[NCLASSES];
//       testImages = new ImageGroup[NCLASSES];
//       integralImages = new VectorOfIntegralImages[NCLASSES];
      images = 0;
      testImages = 0;
      nodes_nos_faces = 1;
      nodes_nos_nonfaces = 1;
      
//       testIntegralImages = new VectorOfIntegralImages[NCLASSES];
      globalNodeIndex = 0;
      windows.clear();
      forestWindows.clear();
      globalDetectionsFile.open("all-detections.txt");
      logfile.open("logfile.txt");
      detectionsFile.open("the-detections.txt");
      faceMapFile.open("the-faceMapFile.txt");
	  integralImages = new VectorOfIntegralImages[ number_of_classes ];
	  testIntegralImages = new VectorOfIntegralImages[ number_of_classes ];
	  nclasses = number_of_classes; 
    }

    /** set the number of identities for leanring 
     * \param n interger for number of classes */ 
	void setNclasses( int n ) {
	  nclasses = n;
	}
    /** get the number of identities used in learning */ 
	int getNclasses() const {
	  return nclasses; 
	}
    /** detection window heurisitc - defining neighbor range 
     * \param p is the range */
    void setSubwindowRange(int p) { subwindow_range = p; }
    /** setting the feature size 
     * \param r is the dimension */ 
    void setRectangularFeatureSize(int r) { rectangular_feature_size = r; }
    /** set fmp gloabl threshold 
     * \param fmp is the value */ 
    void setFaceMapThreshold(double fmp) {face_map_threshold = fmp;}
    /** detection heurisitc range 
     * \param dt is the integeer value */ 
    void setDetectionThreshold(int dt){detection_threshold = dt;} 
    /** set training images location 
     * \param s is the location */ 
    void setTrainImages(string s=TRAIN_IMAGES_LOC){ trainImages_ = s;}
    /** set test images location 
     * \param s is the location */ 
    void setTestImages(string s=TEST_IMAGES_LOC){ testImages_ = s;}
    /** set training images default location 
     * \param s default loc */ 
    void setTrainIntegralImagesLoc( string s=TRAINDT){ trainIntegralImagesLoc_ = s;}
    /** set test images default location 
     * \param s default test images lcoation */
    void setTestIntegralImagesLoc(string s=TESTDT) { testIntegralImagesLoc_ = s;}
    /** set relearn value - offline or online 
     * \param q is 1 for online learning and 0 for offline learning */ 
    void setRelearnStatusValue(bool q) { relearnStatusValue_ = q;}
    /** get subwindow range set */ 
    int getSubwindowRange() const { return subwindow_range; }
    /** get global fmp value */ 
    double getFaceMapThreshold() { return face_map_threshold;}
    /** get detection threshold */ 
    int getDetectionThreshold() { return detection_threshold;}
    /** get training images location */
    string getTrainImages() { return trainImages_;}
    /** get test images location */ 
    string getTestImages() { return testImages_;}
    /** get training images default loc */ 
    string getTrainIntegralImagesLoc(){ return trainIntegralImagesLoc_;}
    /** get test images default loc */ 
    string getTestIntegralImageDataLoc() { return testIntegralImagesLoc_;}
    /** get relearn status value */ 
    bool getRelearnStatusValue() { return relearnStatusValue_;}

    /** destructor */ 
    ~ConfigClass() { 

            
      //      for( int i = 0; i < NCLASSES; i++ ) { 
// 	for(int j = 0; j < images[i].size(); j++ )
// 	  images[i].at( j ) = 0; 
	
// 	images[i].clear(); 
//       }
      
//       delete[] images; 

//       for( int i = 0; i < NCLASSES; i++ ) { 
// 	for( int j = 0; j < testImages[i].size(); j++ ) 
// 	  testImages[i].at( j ) = 0; 
	
// 	testImages[i].clear(); 
//       }

//       delete[] testImages;
      

      for( int i = 0; i < nclasses; i++ ) { 
	for( int j =0; j < integralImages[i].size() ; j++ )
	  delete (integralImages[i].iiVector.at(j)); 
		
//	integralImages[i].clear(); 
      }
	  delete[] integralImages;
	  
      //delete[] integralImages;
      
      for( int i = 0; i < nclasses; i++ ) { 
	for( int j = 0; j < testIntegralImages[i].size(); j++ ) 
	  delete (testIntegralImages[i].iiVector.at(j)); 
	
//	testIntegralImages[i].clear(); 
      }
	  
	  delete[] testIntegralImages; 

      //      if( testIntegralImages!=0 )
      //delete[] testIntegralImages; 
    }      
  };
  
  

  void mergeDetections( vector< WindowClass* >&, ConfigClass& , double scaleFactor ); 

  //merge subwindows as per ~\cite{violaJones2001} detection discussion
  void mergeDetectionSubwindow(std::vector<UserDef::WindowClass*>&, int height, int width, ConfigClass&);

  //This draws tiny circles arounded detected faces/objects as mentioned in the WindowClass vector
  void drawDetections(IplImage* image, std::vector<UserDef::WindowClass *>,  string nameLocOfImage, char* nameAdd, bool saveImages);

  void filterNoise1( UserDef::ConfigClass &, vector< UserDef::WindowClass* > &); 
  void filterNoise2( UserDef::ConfigClass &, vector< UserDef::WindowClass* > &); 
  void filterNoise3( std::vector<UserDef::WindowClass*> &, ConfigClass&, double scaleFactor ); 
  void filterNoise4( UserDef::ConfigClass &, vector< UserDef::WindowClass* > &); 
  void filterNoise5( std::vector<UserDef::WindowClass*> & , ConfigClass&, double scaleFactor , int moduleParameter ); 
  void filterNoise6( std::vector<UserDef::WindowClass*> &, ConfigClass& config); 




}


#endif
