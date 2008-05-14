#include "forest_aux.h" 
#include "forest_param_default.h" 
#include <string> 
#include <iostream> 
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <fstream>  
#include <boost/nondet_random.hpp>
#include <boost/random.hpp>
#include <boost/config.hpp>
#include <boost/random.hpp>
#include <boost/progress.hpp>
#include <boost/shared_ptr.hpp>
#include <dirent.h>
#include <utils/logging/liblogger.h>


/***************************************************************************
 *  forest.h random forest implementation
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


#ifndef __FIREVISION_FVUTILS_RECOGNITION_FOREST_FOREST_H_
#define __FIREVISION_FVUTILS_RECOGNITION_FOREST_FOREST_H_

using namespace std; 
typedef vector<IplImage*> ImageGroup;
boost::minstd_rand g;
boost::uniform_01<boost::minstd_rand> randBoost(g);

/** rand boost */ 
double randomBooster(){
  return randBoost();
}

 
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
 * A config class to maintain the location of training images, the forest size etc. 
 */ 
class ForestConfigClass { 

 

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
  /** limitations of the feature size */ 
  int rectangular_feature_size;
  /** file to store log messages */ 
  std::ofstream logfile;
  /** file to store all detections */
  std::ofstream detectionsFile;
  /** file to store fmp values */ 
  std::ofstream faceMapFile;
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


  /** the integral images of the training samples */ 
  VectorOfIntegralImages *integralImages; 
  /** the integral images of the test sampels */ 
  VectorOfIntegralImages *testIntegralImages; 

  /** maintaining the global node index */ 
  int globalNodeIndex;
    
  /** constructor that takes number of object classes as the parameter
   * @param number_of_classes the number of object classes
   */ 
  ForestConfigClass(int number_of_classes){
    rectangular_feature_size = RECTANGULAR_FEATURE_SIZE;
    images = 0;
    testImages = 0;
    nodes_nos_faces = 1;
    nodes_nos_nonfaces = 1;
    globalNodeIndex = 0;
    globalDetectionsFile.open("all-detections.txt");
    logfile.open("logfile.txt");
    detectionsFile.open("the-detections.txt");
    faceMapFile.open("the-faceMapFile.txt");
    integralImages = new VectorOfIntegralImages[ number_of_classes ];
    testIntegralImages = new VectorOfIntegralImages[ number_of_classes ];
    nclasses = number_of_classes; 
  }

  /** set the number of identities for leanring 
   * @param n interger for number of classes */ 
  void setNclasses( int n ) {
    nclasses = n;
  }
  /** get the number of identities used in learning */ 
  int getNclasses() const {
    return nclasses; 
  }
  /** detection window heurisitc - defining neighbor range 
   * @param p is the range */
  void setSubwindowRange(int p) { subwindow_range = p; }
  /** setting the feature size 
   * @param r is the dimension */ 
  void setRectangularFeatureSize(int r) { rectangular_feature_size = r; }
  /** set fmp gloabl threshold 
   * @param fmp is the value */ 
  void setFaceMapThreshold(double fmp) {face_map_threshold = fmp;}
  /** detection heurisitc range 
   * @param dt is the integeer value */ 
  void setDetectionThreshold(int dt){detection_threshold = dt;} 
  /** set training images location 
   * @param s is the location */ 
  void setTrainImages(string s=TRAIN_IMAGES_LOC){ trainImages_ = s;}
  /** set test images location 
   * @param s is the location */ 
  void setTestImages(string s=TEST_IMAGES_LOC){ testImages_ = s;}
  /** set training images default location 
   * @param s default loc */ 
  void setTrainIntegralImagesLoc( string s=TRAINDT){ trainIntegralImagesLoc_ = s;}
  /** set test images default location 
   * @param s default test images lcoation */
  void setTestIntegralImagesLoc(string s=TESTDT) { testIntegralImagesLoc_ = s;}
  /** set relearn value - offline or online 
   * @param q is 1 for online learning and 0 for offline learning */ 
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

  ~ForestConfigClass() { 

    for( int i = 0; i < nclasses; i++ ) { 
      for( int j =0; j < integralImages[i].size() ; j++ )
	delete (integralImages[i].iiVector.at(j)); 
    }
    delete[] integralImages;
	  
    for( int i = 0; i < nclasses; i++ ) { 
      for( int j = 0; j < testIntegralImages[i].size(); j++ ) 
	delete (testIntegralImages[i].iiVector.at(j)); 
    }
	  
    delete[] testIntegralImages; 

  }      
};
  
/** random x index 
 * @param config the config class instance 
 */
int randRowIndex( ForestConfigClass &config )
{ 
  return rand()%config.imageInfoInstance.getHeight(); 
}
/** random y index
 * @param config the config class instance 
 */
int randColIndex( ForestConfigClass &config )
{ 
  return rand()%config.imageInfoInstance.getWidth(); 
}

/**
 * Test candidate for the forest 
 */ 
class ForestTest { 

  
 public:
  /** all images that get assigned on left brnach on test candidate */ 
  VectorOfIntegralImages *leftBranch; 
  /** all images that get assigned to rifght branch on test candidate */ 
  VectorOfIntegralImages *rightBranch; 
  /** the threshold theta1, */
  double theta1; 
  /** theta2 */
  double  theta2;
  /** top left coordinates of randomrectangle sampled */ 
  int xMain; 
  /** top left coordinates of randomrectangle sampled */    
  int yMain;
    
  /** width of the sampled random rectangle */ 
  int xdash; 
  /** height of the sampled random rectangle */     
  int ydash;

  /** haar featuree considered */ 
  int haarFeature;
	
	
 private:
	
  /** entropy value of the test candidate */ 
  double entropyValue; // the value of the entropy between the left and right branches.
    
  /** 
   * calculation of entropy using class histogram and derived probabiliities 
   */ 
  double entropyCalculation(ForestConfigClass& config)
  {
    double finalProbabilityLeft = 0.0;
    double finalProbabilityRight = 0.0;
    double probabilityLeft = 0.0, probabilityRight = 0.0;
    int numeratorLeft = 0, numeratorRight = 0;
    int denominatorLeft = 0;
    int denominatorRight = 0;
    int i=0;
	  
    for(i=0;i<config.getNclasses();i++)
      {
	denominatorLeft += leftBranch[i].iiVector.size()+1;
	denominatorRight += rightBranch[i].iiVector.size()+1;
      }
	  
    for(i=0;i<config.getNclasses();i++)
      {
	numeratorLeft = leftBranch[i].iiVector.size()+1; 
	numeratorRight = rightBranch[i].iiVector.size()+1;
	probabilityLeft = (double) numeratorLeft/denominatorLeft;
	probabilityRight = (double) numeratorRight/denominatorLeft;
		
	finalProbabilityLeft += probabilityLeft*log2(probabilityLeft);
	finalProbabilityRight += probabilityRight*log2(probabilityRight);
      }
    finalProbabilityLeft *=-1.0;
    finalProbabilityRight *=-1.0;
	  
    if(finalProbabilityLeft > finalProbabilityRight) // en.wikipedia.org/wiki/Shannon_entropy
      return (entropyValue = finalProbabilityLeft);
    else
      return (entropyValue = finalProbabilityRight);
	  
  }
	
 public:
  /** 
   * get entropy for split by test candidate
   * @param config ConfigClass Instance 
   */ 
  double getEntropyValue(ForestConfigClass &config) { return (entropyValue=entropyCalculation(config));}


  /** 
   * a constructor 
   * @param copyTest created to copy a chosen test candidate and store it at the node
   * @param config ConfigClass Instance 
   */

  ForestTest(ForestTest* copyTest, ForestConfigClass& config)
    {
	  
      leftBranch = new VectorOfIntegralImages[ config.getNclasses() ];
      rightBranch = new VectorOfIntegralImages[ config.getNclasses() ]; 
      xdash = ydash = xMain = yMain = haarFeature = 0;
      theta1 = theta2 = 0.0; 

      for(int i=0;i<config.getNclasses();i++){
		
	for(unsigned int j=0; j<copyTest->leftBranch[i].iiVector.size(); j++)
	  leftBranch[i].iiVector.push_back( copyTest->leftBranch[i].iiVector.at(j) );
		
		
	for(unsigned int j=0; j<copyTest->rightBranch[i].iiVector.size(); j++)
	  rightBranch[i].iiVector.push_back( copyTest->rightBranch[i].iiVector.at(j) );
      }
	  
      theta1 = copyTest->theta1;
      theta2 = copyTest->theta2;
      xdash = copyTest->xdash;
      ydash = copyTest->ydash;
      xMain = copyTest->xMain;
      yMain = copyTest->yMain;
	  
      entropyValue = copyTest->getEntropyValue(config);
      haarFeature = copyTest->haarFeature;
    }

  /** 
   * a constructor that works form the configClass object
   * @param config ConfigClass Instance
   */ 
 ForestTest(ForestConfigClass& config):/*leftBranch(0), rightBranch(0), */theta1(0.0), theta2(0.0), xMain(0), 
    yMain(0), xdash(0), ydash(0), haarFeature(0) {
    leftBranch = new VectorOfIntegralImages[ config.getNclasses() ];
    rightBranch = new VectorOfIntegralImages[ config.getNclasses() ];
  }

  /** 
   * a constructor that takes in the top left coordinates of the random rectangle 
   * @param x top left x coord
   * @param y top left y coord 
   * @param config ConfigClass Instance 
   */ 
 ForestTest(int x, int y, ForestConfigClass& config): theta1(0.0), theta2(0.0), xMain(x),yMain(y), xdash(0), ydash(0),
    haarFeature(0)	 {
	  
    leftBranch = new VectorOfIntegralImages[ config.getNclasses() ];
    rightBranch = new VectorOfIntegralImages[ config.getNclasses() ];
	  
    theta1 =randomBooster();
    theta2 =randomBooster();
	  
    while( (xdash = randomRectangle()) < RECTANGULAR_MIN_FEATURE_SIZE ) 
      xdash = randomRectangle();
	  
    while( (ydash = randomRectangle()) < RECTANGULAR_MIN_FEATURE_SIZE )
      ydash = randomRectangle();
	  
	  
    double temp;
    if(theta1>theta2) { 
      temp = theta1;
      theta1 = theta2;
      theta2 = temp;
    }
	  
    entropyValue = -1.0;
  }


  /** Does the image pass the test - as defined by a region and threshold? 
   * \param integralImage - the integral image of the test image
   * \param xMain top left x coordinate 
   * \param yMain top left y coordinate 
   * \param xdash width 
   * \param ydash height 
   * \param theta1 first threshold 
   * \param theta2 second threshold 
   * \param height height of the training images 
   * \param width width of the training images 
   * \param haarFeature haar feature taken into consideration 
   */

  bool imagePass(int* integralImage, int xMain, int yMain, int xdash, 
		 int ydash, double theta1, double theta2, int height, int width, int haarFeature){
	  
    return featurePass(
		       getRectangularIntegralImagefeature(
							  integralImage, xMain, yMain, 
							  xdash, ydash, height, width, 
							  haarFeature), 
		       theta1, theta2, 
		       NOS_THRESHOLDS);
  }

  /** Run tests on the integral images and return final entropy of the division. 
   * @param integralImages vector of II 
   * @param haarFeature haar feature consdiered
   * @param config ConfigClass Instance
   */
  double runTestsOnIntegralImages(VectorOfIntegralImages *integralImages, 
				  int haarFeature, ForestConfigClass& config){
	  
	  
    for(int i=0;i<config.getNclasses();i++){
      leftBranch[i].iiVector.clear();
      rightBranch[i].iiVector.clear();
    }
	  
    this->haarFeature = haarFeature;
	  
    int *integralImage;
	  
    for(int i=0;i<config.getNclasses();i++)
      for(unsigned int j=0;j<integralImages[i].iiVector.size();j++){
	integralImage = (integralImages[i].iiVector.at(j));
		  
	if(imagePass((integralImage), xMain, yMain, xdash, ydash, theta1, 
		     theta2, config.imageInfoInstance.getHeight(), 
		     config.imageInfoInstance.getWidth(), haarFeature))
	  leftBranch[i].iiVector.push_back(integralImages[i].iiVector.at(j));
	else
	  rightBranch[i].iiVector.push_back(integralImages[i].iiVector.at(j));
      }
	  
    integralImage = 0; 					
	  
	  
    return getEntropyValue(config);
  }
	
	
  /** destrcutor for test */ 	
  ~ForestTest()
    {
	  
      delete[] leftBranch;
      delete[] rightBranch;

    }
	
};
  
/**
 * Builds a random tree by instantiating L number of tests
 */ 
class Tree {
 private:
  /** level of the tree */ 
  int TreeLevel; // was initially static int TreeLevel
  /** index of the forest */ 
  int ForestIndex; // Useful to control the forest; has a value of -1 if the tree is not Root
	
 public:
  /** vector of integral images for the tree */ 
  VectorOfIntegralImages *theIntegralImages; // [config.getNclasses()];
  /** entropy of the parent node for comparisons */
  double parentEntropy_;
  /** index of the current node */ 
  int nodeIndex_;
  /** index of the parent node */ 
  int parentNodeIndex_;
  /** level of the node in the main tree */
  int level_;
  /** best candidate chosen */ 
  ForestTest* bestTest;
  /** link the left child */
  Tree* leftTree;
  /** link to the right child */
  Tree* rightTree;
  /** how difficult is to split the images at the node? */ 
  int max_try; 
	
  /** get the level of the tree */ 
  int getTreeLevel() const { return TreeLevel; }
	
  /** is the node allowed to grow further? */
  bool isNodeAllowed() const { return ( max_try > MAX_TRY_ALLOWED )?true:false; }
	
  /** are the images at the node difficult to divide here? */ 
  void nodeTry() { max_try++; }
	
  /** what is the index of the current random tree */
  int getForestIndex() const { return ForestIndex; }
	
  /** get a unique number to label the nodes of the random tree */ 
  int getUniqueNumber() const 
  {
    return nodeIndex_;   
  }
	
  /** increment the level of the tree growth */
  int incrementLevel() { return TreeLevel++; }
	
  /** destructor */
  ~Tree()
    {
	  
      delete bestTest; 
      delete leftTree; 
      delete rightTree; 
      delete[] theIntegralImages; 
	
    }


  /** Tree constructor for all nodes other than the "main" root node 
   * @param integralImages vector of integral images at the node 
   * @param pE entropy of the parent node 
   * @param levelOfTree level of the node in the main tree 
   * @param sNodeIndex node index of the current node 
   * @param sParentNodeIndex node index of the parent node
   * @param config ConfigClass Instance 
   */
 Tree(VectorOfIntegralImages* integralImages, double pE, int levelOfTree, 
      int sNodeIndex, int sParentNodeIndex, ForestConfigClass &config): 
  ForestIndex(-1),parentEntropy_(pE),  nodeIndex_(sNodeIndex), 
    parentNodeIndex_(sParentNodeIndex), level_(levelOfTree), 
    bestTest(0), leftTree(0), rightTree(0), max_try(0)  {
	  
    theIntegralImages = new VectorOfIntegralImages[config.getNclasses()];
	  

    for(int i=0;i<config.getNclasses();i++)
      theIntegralImages[i].iiVector.assign(integralImages[i].iiVector.begin(), 
					   integralImages[i].iiVector.end());
	 
  }



  /** Tree constructor for "only" the root node 
   * @param config ConfigClass Instance
   * @param globalNodeIndex global counter to monitor node indices
   * @param ForestIndexOfRoot number of the random tree in the forest 
   */ 
 Tree( ForestConfigClass &config, int &globalNodeIndex, 
       int ForestIndexOfRoot): 
  ForestIndex(ForestIndexOfRoot), parentEntropy_(200.0), 
    nodeIndex_(globalNodeIndex++), parentNodeIndex_(-1), 
    level_(0), bestTest(0), leftTree(0), 
    rightTree(0), max_try(0)
    {
	  
      theIntegralImages = new VectorOfIntegralImages[ config.getNclasses() ];
	 
	  
      for( int i =0; i < config.getNclasses(); i++ )
	theIntegralImages[i].iiVector.assign( 
					     config.integralImages[i].iiVector.begin(), 
					     config.integralImages[i].iiVector.end() ); 
      //		 } 
	  
	  
      level_ = incrementLevel();
	  
      ofstream graphviz;
      string treedot = TREEDOT;
      char *buffer, *bins, *buffer1; 
      int sizeOfSprintf = 0 ; 
      ForestTest* temp_test = 0; 
	  
	  
      if( WR_DOT_FILE ) 
	{  // if GRAPHVIZ output is necessary 
	  graphviz.open(treedot.c_str());
	  buffer = new char[ 300 ];
	  bins = new char[ 200 ];
	  buffer1 = new char[ 50 ]; 
		
	  int sizeOfSprintf = sprintf(buffer,"digraph g { node [shape = record,height=.1];"); 
	  graphviz.write(buffer, sizeOfSprintf);
	}
	  
      std::vector< ForestTest* >  testsMade; // the 200 or 'N' number of tests created
      int finalIndex = -1;  // the index of the best test: greatest entropy value
      double* entropies = 0 ;
      entropies = new double[NITERATIONS];
	  
      int i=0, temp=0;
      int haarIndex = 0;
      double entropiesHaar[HAARFEATURES];
      int testIndex[HAARFEATURES];
	  
      for(i =0;i<NITERATIONS;i++)  // make tests
	{ 
	  temp_test = new ForestTest(randRowIndex(config),randColIndex(config), config); 
	  testsMade.push_back( temp_test );
	} 
	  
      /* I am going to run the tests with each of the Haar features */
      for(haarIndex = 0; haarIndex < HAARFEATURES; haarIndex++ )
	{
	  for(i=0;i<NITERATIONS;i++)
	    entropies[i] = testsMade.at(i)->runTestsOnIntegralImages( theIntegralImages, haarIndex, config );
		
	  finalIndex = findMin( entropies, NITERATIONS );
	  entropiesHaar[haarIndex] = entropies[finalIndex];
	  testIndex[haarIndex] = finalIndex;
	}	
	  
	  
      /* 
	 find min of the best entropies for each of the haar features - 
	 so in essence, haarIndex has the best Haar feature for the 
	 200 tests considered
      */
      haarIndex = findMin(entropiesHaar, HAARFEATURES);
      /*
	re-use final index to store the index of the best test 
	( a test is instantiated by xMain,YMain,xdash, ydash,
	theta1, theta2 & a HAAR feature found out here. So, we keep
	track of the other things 
      */
      finalIndex = testIndex[haarIndex];
      /* 
	 since the allocation of images will be different in the
	 given testsMade vector (it will be of the last test run) , we run it again 
      */  
      testsMade.at(finalIndex)->runTestsOnIntegralImages(theIntegralImages, haarIndex, config);
      bestTest = new ForestTest(testsMade.at(finalIndex), config);
	  
      for( unsigned int k = 0; k < testsMade.size(); k++ ) 
	{ 
	  temp_test = testsMade.at( k ); 
	  delete temp_test; 
	}         
	  
      temp = 0; 
      testsMade.clear();
	  
      if(WR_DOT_FILE) {		   /* graphviz output of the root being divided */
	strcpy(bins," ");
	for(i=0;i<config.getNclasses()-1;i++){
	  sprintf(buffer1,"%zu", theIntegralImages[i].iiVector.size());		  
	  strcat(bins, buffer1);
	  strcat(bins,",");
	}
	sprintf(buffer1,"%zu",theIntegralImages[config.getNclasses()-1].iiVector.size());		  
	strcat(bins, buffer1);
	sprintf(buffer1, "%f",bestTest->getEntropyValue(config));
	sizeOfSprintf = sprintf(buffer,
				"node%d[label = \"<f0> %d| <f1> %s |<f2> %s \"];",
				nodeIndex_,nodeIndex_,bins,
				buffer1);
	graphviz.write(buffer, sizeOfSprintf);
		
      }
	  
      /* 
	 We want to run the same 200 tests on every child at a level,
	 so there is a vector containing these children (initialized)
      */
      /* these hold Trees to be tested now */
      vector<Tree*> childrenToTest; 
      /* these hold the left and right branches of the Trees being 
	 tested now ( they are to be tested in the next iteration ) 
      */
      vector<Tree*> tempChildrenToTest; 
	  
      /* now, the left and right nodes of the root is going to be created. 
	 Hence, we increment the level of the tree depth 
      */
      i = incrementLevel(); 
	  
      /* left tree obtains the left branch of the best split. */
      leftTree = new Tree(bestTest->leftBranch, bestTest->getEntropyValue(config), 
			  i, globalNodeIndex++ , nodeIndex_, config );
	  
      if( WR_DOT_FILE ) { 
		
	/* grapbviz for left tree */
	sizeOfSprintf = sprintf(buffer,"\"node%d\":f0 -> \"node%d\":f1;",nodeIndex_,leftTree->nodeIndex_);
	graphviz.write(buffer, sizeOfSprintf);
      }
	  
      /* right tree obtains the right branch of the best split */
      rightTree = new Tree(bestTest->rightBranch, bestTest->getEntropyValue(config), 
			   i, globalNodeIndex++, nodeIndex_, config );
	  
      if( WR_DOT_FILE ) { 
		
	/* graphviz for the right tree */
	sizeOfSprintf = sprintf(buffer,"\"node%d\":f2 -> \"node%d\":f1;",nodeIndex_,rightTree->nodeIndex_);
	graphviz.write(buffer, sizeOfSprintf);
      }	   
	  
      /* 
	 clear vector of the children to start splitting. 
	 (in case of garbage values) 
      */
      childrenToTest.clear();
      /* put the left and right trees on the stack */
      childrenToTest.push_back(leftTree);
      childrenToTest.push_back(rightTree);
      /* clear another vector (clear out garbage) */
      tempChildrenToTest.clear();
	  
	  
      while(globalNodeIndex<NODECOUNT && childrenToTest.size()!=0){		  
	/* we want only NODECOUNT to be total number of nodes created */
		
		
		
	for(i=0;i<NITERATIONS;i++) { 
	  temp_test = new ForestTest(randRowIndex(config),randColIndex(config), config); 
	  testsMade.push_back( temp_test );
	}
		
		
	    for(int j=0;j<(int)childrenToTest.size();j++){
		  
	      /* run tests for the images for the jth child */
	      for(haarIndex = 0; haarIndex<HAARFEATURES; haarIndex++){
		for( i=0;i < NITERATIONS; i++)
		  entropies[i] = testsMade.at(i)->
		    runTestsOnIntegralImages(childrenToTest.at(j)->theIntegralImages, 
					     haarIndex, config);
			
			
		finalIndex = findMin(entropies, NITERATIONS);
		entropiesHaar[haarIndex] = entropies[finalIndex];
		testIndex[haarIndex] = finalIndex;
	      }
		  
		  
	      haarIndex = findMin(entropiesHaar, HAARFEATURES);
	      finalIndex = testIndex[haarIndex];
		  
	      testsMade.at(finalIndex)->
		runTestsOnIntegralImages(childrenToTest.at(j)->theIntegralImages, 
					 haarIndex, config);
		  
	      childrenToTest.at(j)->bestTest = new ForestTest(testsMade.at(finalIndex), config);
		  
	      /* graphviz instructions - create the node (label it)*/
	      if( WR_DOT_FILE )
		strcpy(bins," ");
		  
	      for(i=0;i<config.getNclasses()-1;i++){
			
		if( WR_DOT_FILE )
		  sprintf(buffer1,"%zu",childrenToTest.at(j)->theIntegralImages[i].iiVector.size());
		if( WR_DOT_FILE ) { 
		  strcat(bins, buffer1);
		  strcat(bins,",");
		}
	      }
		  
	 
		  
	      if(WR_DOT_FILE) { 
		sprintf(buffer1,"%zu",childrenToTest.at(j)->theIntegralImages[config.getNclasses()-1].iiVector.size());
		strcat(bins, buffer1);
		sprintf(buffer1, "%f",childrenToTest.at(j)->bestTest->getEntropyValue(config));
		sizeOfSprintf = sprintf(buffer,
					"node%d[label = \"<f0> %d| <f1> %s |<f2> %s \"];", 
					childrenToTest.at(j)->nodeIndex_, 
					childrenToTest.at(j)->nodeIndex_,bins,buffer1);
		graphviz.write(buffer, sizeOfSprintf);
			
	      }
		  
	      //		if(!checkGrowth){
		  
	      bool leftBranchIsEmpty = true;
	      bool rightBranchIsEmpty = true;
		  
	      for( int k = 0 ; k < config.getNclasses() ; k++ )
		if( testsMade.at( finalIndex )->leftBranch[ k ].size() != 0 ) {
		  leftBranchIsEmpty = false; 
		}
		  
		  
	      for( int k = 0 ; k < config.getNclasses() ; k++ ) 
		if( testsMade.at( finalIndex )->rightBranch[ k ].size() != 0 ) { 
		  rightBranchIsEmpty = false; 
		}
		  
		  
		  
	      if( !leftBranchIsEmpty )
		childrenToTest.at(j)->leftTree = new Tree(testsMade.at(finalIndex)->leftBranch, 
							  entropiesHaar[haarIndex], temp, 
							  globalNodeIndex++, 
							  childrenToTest.at(j)->nodeIndex_, config);
		  
	      if( WR_DOT_FILE && !leftBranchIsEmpty ) 
		{ 
		  /* establish graphviz connection between parent & this new child */
		  sizeOfSprintf = sprintf(buffer,"\"node%d\":f0 -> \"node%d\":f1;",
					  childrenToTest.at(j)->nodeIndex_, 
					  childrenToTest.at(j)->leftTree->nodeIndex_);
		  graphviz.write(buffer, sizeOfSprintf);
		}
		  
		  
	      if( !rightBranchIsEmpty )
		childrenToTest.at(j)->rightTree = new Tree(testsMade.at(finalIndex)->rightBranch, 
							   entropiesHaar[haarIndex], temp, 
							   globalNodeIndex++, 
							   childrenToTest.at(j)->nodeIndex_, config);
		  
	      if( WR_DOT_FILE && !rightBranchIsEmpty ) 
		{ 
		  /* establish graphviz connection between parent & new child */
		  sizeOfSprintf = sprintf(buffer,"\"node%d\":f2 -> \"node%d\":f1;",
					  childrenToTest.at(j)->nodeIndex_,
					  childrenToTest.at(j)->rightTree->nodeIndex_);
			
		  graphviz.write(buffer, sizeOfSprintf);
			
		}
		  
		  
	      /* store the newly created trees for tests in the next iteration */
		  
	      if( !leftBranchIsEmpty )
		tempChildrenToTest.push_back(childrenToTest.at(j)->leftTree);
	      if( !rightBranchIsEmpty )
		tempChildrenToTest.push_back(childrenToTest.at(j)->rightTree);
	
	    }
		
		
		
	    ForestTest* temp_test = 0;
	    for( unsigned int k = 0; k < testsMade.size() ; k++ ) { 
	      temp_test = testsMade.at( k ); 
	      delete temp_test; 
	    }
		
		
	    temp_test = 0; 
		
	    testsMade.clear();
	    childrenToTest.clear(); 
	    childrenToTest.assign(tempChildrenToTest.begin(), tempChildrenToTest.end());
	    tempChildrenToTest.clear();
		
	  }
	  
	  delete[] entropies;	   
	  entropies = 0;
	  
	  if(WR_DOT_FILE) {
	    graphviz.write("}",1);
	    graphviz.close();  
	  }
	} 
  }; 


 /** grow a random tree quickly, given the location of the training samples 
   * @param train_images images from the training samples
   * @param nclass number of identities 
   * @param root root node of the random tree
   * @param train_height height of the training samples
   * @param train_width width of the training images
   * @param config ConfigClass Instance 
*/ 
  void growTreeFromImages( char* train_images, int nclasses, Tree* &root, int &train_height, int &train_width, ForestConfigClass& config  )
  {
    // Define a ConfigClass
	
    config.setSubwindowRange( SUBWINDOW_RANGE ); 
    config.setTrainImages( train_images ); 

    IplImage* tempImage = 0; 
    DIR* dir = 0;
	
    // Holds file name for reading images. 
    char itoaString[20];
	
    int i = 0, j = 0;
    // Set to false if loaded file is not an image.
    bool flagNotImage = false;
    struct dirent *ent = 0;
    int *tempII;
	
    string fileName , dirPath, mainDirPath;

    mainDirPath = config.getTrainImages(); 
	
    for(i = 0;i<config.getNclasses();i++){
	  
      j = 0;
	  
      sprintf(itoaString,"%d",i);
		
      dirPath = mainDirPath + itoaString;
		
      if((dir = opendir(dirPath.c_str())) == NULL){
	printf("the directory searched for is %s\n", dirPath.c_str() ); 
	LibLogger::log_error("Forest","The directory does not exist");
	continue;
	//		  perror("Directory does not exist");
	//		  exit(0);
      }
		
      bool setBorders = false; 
		
      while((ent = readdir(dir))!=NULL){
	fileName = dirPath + "/" + ent->d_name;
		  
	tempImage = cvLoadImage(fileName.c_str());
		  
	if(!tempImage){
	  flagNotImage = true;
	}
		  
	if( !flagNotImage ) { 
	  j++;
			
	  //	  config.images[i].push_back(tempImage);
			
	  config.imageInfoInstance.setHeight( tempImage->height );
	  config.imageInfoInstance.setWidth( tempImage->width );
			
	  train_height = tempImage->height; 
	  train_width = tempImage->width; 
			
	  tempII = new int[ ( tempImage->height ) * ( tempImage->width ) ];
			
	  calculateIntegralImage( tempImage, tempII );
			
	  config.integralImages[i].iiVector.push_back( tempII );
			
	  if( !setBorders ) { 
			  
	    config.integralImages[i].setHeight( tempImage->height ); 
	    config.integralImages[i].setWidth( tempImage->width ); 
	    setBorders = true; 
	  }
			
			
	  cvReleaseImage( &tempImage ); 
	}  
		  
	flagNotImage = false;
      } 
    } 	  
	  
    int globalNodeIndex = 0;
    int numberOfTreesGrown = 0;
    root = new Tree( config, globalNodeIndex, numberOfTreesGrown++ ); 

		
  }


/** 
 * Class to create a random forest transparently. 
 * Can be used in demo mode or an actual instantiation. 
 * Includes most of the parameterization
 */
class ForestClass 
{
  
 private: 
  /** the random forest */
  vector<Tree*> _forest; 
  /** number of identiites */ 
  int _nclasses; 
  /** the size of the forest */
  int _forestSize; 
  /** the height of the training images */ 
  int _train_height; 
  /** the width of the trainign images */ 
  int _train_width; 

 public:     
  /** constructor 
   * @param train_images the training samples 
   * @param nclasses number of identities 
   * @param train_height height of the training samples 
   * @param train_width widht of the trainig images 
   * @param config ConfigClass Instance 
   * @param forestSize size of the forest 
   */ 
  ForestClass( char* train_images, int nclasses, int &train_height, int &train_width, ForestConfigClass& config, int forestSize ) 
    { 
    
      _nclasses = nclasses; 
      _forestSize = forestSize; 

    
      for(int i = 0; i < _forestSize ; i++ ) 
	{ 

   
	  int _globalNodeIndex = 0;
	  int _numberOfTrees = 0;
	  if( i == 0 )
	    {   
	      Tree* tempTree; 
	      growTreeFromImages( train_images, nclasses, tempTree, train_height, train_width, config ); 
	      _train_height = train_height; 
	      _train_width = train_width; 
	      _forest.push_back( tempTree ); 
	    }
    
	  else 
	    {
	      _globalNodeIndex = 0 ;
	      _numberOfTrees = i; 
	      _forest.push_back( new Tree( config, _globalNodeIndex, _numberOfTrees ) ); 
	    }
    
    
	}
   
      
    }
   

  /** get the forest size */ 
  int getSize() const 
  { 
    return _forest.size(); 
  }
    
  /** get the numner of obeject classes/identities */ 
  int getNclasses() const 
  { 
    return _nclasses; 
  }
    
  /** get the height of the training samples - for rescaling test images */ 
  int getTrainHeight() const 
  { 
    return _train_height; 
  }
     
  /** get the width of the training samples - for rescaling test images */ 
  int getTrainWidth() const 
  { 
    return _train_width;
  }
       


  /** get the random tree of index i from the forest 
   * @param i get the tree of the index in the forest 
   */ 
  Tree* getTree( int i ) 
  { 
  
    return this->_forest.at(i);
  }


  /** destrcutor */    
  ~ForestClass() 
    {

      for( unsigned int i = 0; i < _forest.size(); i++ ) 
	delete _forest.at(i); 

    } 
};
  	


/** get classification results for any random tree 
 * @param aTree the random tree
 * @param ii the integral image
 * @param train_height height of the training images
 * @param train_width widht of the trainign images
 * @param nclasses number of indentities 
 */ 
int runClassificationForArbitraryTree( Tree* aTree, int* &ii, int train_height, int train_width, int nclasses )
{
	
  Tree *root = aTree, *prevTree = aTree; 
  bool childPass; 
  ForestTest* root_test = 0;
	
 	
  while( root!= 0 )
    {
      if( root->leftTree == 0 || root->rightTree == 0 )
	{
	  if( root->theIntegralImages == 0 ) 
	    {
	      root = prevTree;
	      break;
	    }
	  else
	    break;
	}
	  
      root_test = root->bestTest; 
	  
      if( root_test == 0 )
	{
	  root = prevTree; 
	  break; 
	}
	  
      childPass = root_test->imagePass( ii, root_test->xMain, root_test->yMain, root_test->xdash, root_test->ydash, root_test->theta1, root_test->theta2, train_height, train_width, root_test->haarFeature ); 
	  
      prevTree = root; 
	  
      if( childPass ) 
	root = root->leftTree; 
      else
	root = root->rightTree; 
	  
    }
	
  int max = -1; 
  int index = -2; 
  int total_size = 0; 
	
  for( int i = 0; i < nclasses; i++ ) 
    {
	  
      if( (int)(root->theIntegralImages[i].iiVector.size()) > max ) 
	{
	  max  = root->theIntegralImages[i].iiVector.size();
	  index = i; 
		
	  total_size += root->theIntegralImages[i].iiVector.size(); 
	}
	  
    }
	
  double probabilityOfHighestClass = (double)max/(double)total_size; 
	
  if( probabilityOfHighestClass > IDENTITY_THRESHOLD )
    return index; 
	
  else
    return -1; 
	
	
}


/** return the classsification results for an IplImage -i.e. integral image calculated here 
 * @param aTree the random tree
 * @param imageToClassify the image to classify/recognize
 * @param train_height height of the training images
 * @param train_width width of the training images
 * @param nclasses number of identities 
 */ 
int returnClassLabelForIplImage( Tree *aTree, IplImage *imageToClassify, int train_height, int train_width, int nclasses )
{
	
  int* ii = new int[ imageToClassify->height * imageToClassify->width ]; 
  calculateIntegralImage( imageToClassify, ii );
	
  return runClassificationForArbitraryTree( aTree, ii, train_height, train_width, nclasses  ) ; 
	

}

  
/** for a given forest class instance return the classification label. 
 * @param forestClassInstance the forest class instance
 * @param inputImage the input image
 */ 
int getClassLabelFromForest( ForestClass* forestClassInstance , IplImage* inputImage )
{
    
  int recognition_histogram[ forestClassInstance->getSize() ];
    
  for( int i = 0; i < forestClassInstance->getSize(); i++ ) 
    recognition_histogram[i] = returnClassLabelForIplImage( forestClassInstance->getTree(i), inputImage, forestClassInstance->getTrainHeight(), forestClassInstance->getTrainWidth(), forestClassInstance->getNclasses() );  
      
  int votes[ forestClassInstance->getNclasses() ]; 
    
  for( int i = 0; i < forestClassInstance->getNclasses(); i++ ) 
    votes[i] = 0;
        
  for( int i = 0; i < forestClassInstance->getSize(); i++ ) 
    votes[ recognition_histogram[i] ]++; 
        
  int max_identity = -1; 
  int index_max_identity = -1; 
	  
  for( int i = 0; i < forestClassInstance->getNclasses() ; i++ ) 
    if( votes[i] > max_identity ) 
      {
	max_identity = votes[i]; 
	index_max_identity = i; 
      }
	  

  return index_max_identity; 
    
}
	
	

#endif
