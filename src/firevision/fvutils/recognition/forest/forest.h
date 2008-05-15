/***************************************************************************
 *  forest.h random forest implementation
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


#ifndef __FIREVISION_FVUTILS_RECOGNITION_FOREST_FOREST_H_
#define __FIREVISION_FVUTILS_RECOGNITION_FOREST_FOREST_H_

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


using namespace std; 
typedef vector<IplImage*> ImageGroup;
boost::minstd_rand g;
boost::uniform_01<boost::minstd_rand> randBoost(g);

/** rand boost */ 
double random_booster(){
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
  void set_height(int h){height=h;}
  /** set width information 
   * \param w integer denoting width
   */ 
  void set_width(int w){width = w;}
  /** get the height set */
  int get_height() const { return height;}
  /** get the width considered */ 
  int get_width() const {return width;}
};

/** 
 * A config class to maintain the location of training images, the forest size etc. 
 */ 
class ForestConfigClass { 

 

 private:
  /** the training images location */ 
  string __train_images;
  /** the test images location */ 
  string __test_images;
  /** the global fmp  value*/ 
  int __detection_threshold; 
  /** detection heuristic */ 
  int __subwindow_range;
  /** default location of the training images */ 
  string __train_integral_images_loc;
  /** default location of the test images */ 
  string __test_integral_images_loc; 
  /** fmp value global threshold */ 
  double __face_map_threshold; 
    

 public:
  /** number of the identities */ 
  int nclasses; 
  /** offline or online training */
  bool relearn_status_value; 
  /** Given the sliding window implementation, imageInfoInstance refers to size that the tree was trained with (32X32 and so on). */
  ImageInfoClass image_info_instance; 
  /** testImageInfoInstance details the test Image submitted. Essentially, we look for faces of the size we trained the tree with(32X32) which are the dimensions of the window as well in the whole test image. */
  ImageInfoClass test_image_info_instance; 
  /** limitations of the feature size */ 
  int rectangular_feature_size;
  /** file to store log messages */ 
  std::ofstream log_file; 
  /** file to store all detections */
  std::ofstream detections_file;
  /** file to store fmp values */ 
  std::ofstream face_map_file; 
  /** the trainig images */ 
  ImageGroup* images;
  /** file to store all the detection results */ 
  ofstream global_detections_file; 
  /** image grop of the test images */ 
  ImageGroup* test_images;
  /** histogram for the entire forest  */ 
  int nodes_nos_faces; // nodes_nos_faces, nodes_nos_nonfaces count the number of faces in all the nodes all the path
  /** histogram for the entire forest */
  int nodes_nos_nonfaces; 


  /** the integral images of the training samples */ 
  VectorOfIntegralImages *integral_images; 
  /** the integral images of the test sampels */ 
  VectorOfIntegralImages *test_integral_images; 

  /** maintaining the global node index */ 
  int global_node_index; 
    
  /** constructor that takes number of object classes as the parameter
   * @param number_of_classes the number of object classes
   */ 
  ForestConfigClass(int number_of_classes){
    rectangular_feature_size = RECTANGULAR_FEATURE_SIZE;
    images = 0;
    test_images = 0;
    nodes_nos_faces = 1;
    nodes_nos_nonfaces = 1;
    global_node_index = 0; 
    global_detections_file.open("all-detections.txt");
    log_file.open("logfile.txt");
    detections_file.open("the-detections.txt");
    face_map_file.open("the-faceMapFile.txt");
    integral_images = new VectorOfIntegralImages[ number_of_classes ];
    test_integral_images = new VectorOfIntegralImages[ number_of_classes ];
    nclasses = number_of_classes; 
  }

  /** set the number of identities for leanring 
   * @param n interger for number of classes */ 
  void set_nclasses( int n ) {
    nclasses = n;
  }
  /** get the number of identities used in learning */ 
  int get_nclasses() const {
    return nclasses; 
  }
  /** detection window heurisitc - defining neighbor range 
   * @param p is the range */
  void set_subwindow_range(int p) { __subwindow_range = p; }
  /** setting the feature size 
   * @param r is the dimension */ 
  void set_rectangular_feature_size(int r) { rectangular_feature_size = r; }
  /** set fmp gloabl threshold 
   * @param fmp is the value */ 
  void set_face_map_threshold(double fmp) {__face_map_threshold = fmp;}
  /** detection heurisitc range 
   * @param dt is the integeer value */ 
  void set_detection_threshold(int dt){__detection_threshold = dt;} 
  /** set training images location 
   * @param s is the location */ 
  void set_train_images(string s=TRAIN_IMAGES_LOC){ __train_images = s;}
  /** set test images location 
   * @param s is the location */ 
  void set_test_images(string s=TEST_IMAGES_LOC){ __test_images = s;}
  /** set training images default location 
   * @param s default loc */ 
  void set_train_integral_images_loc( string s=TRAINDT){ __train_integral_images_loc = s;}
  /** set test images default location 
   * @param s default test images lcoation */
  void set_test_integral_images_loc(string s=TESTDT) { __test_integral_images_loc = s;}
  /** set relearn value - offline or online 
   * @param q is 1 for online learning and 0 for offline learning */ 
  void set_relearn_status_value(bool q) { relearn_status_value = q;}
  /** get subwindow range set */ 
  int get_subwindow_range() const { return __subwindow_range; }
  /** get global fmp value */ 
  double get_face_map_threshold() { return __face_map_threshold;}
  /** get detection threshold */ 
  int get_detection_threshold() { return __detection_threshold;}
  /** get training images location */
  string get_train_images() { return __train_images;}
  /** get test images location */ 
  string get_test_images() { return __test_images;}
  /** get training images default loc */ 
  string get_train_integral_images_loc(){ return __train_integral_images_loc;}
  /** get test images default loc */ 
  string get_test_integral_image_data_loc() { return __test_integral_images_loc;}
  /** get relearn status value */ 
  bool get_relearn_status_value() { return relearn_status_value;}

  ~ForestConfigClass() { 

    for( int i = 0; i < nclasses; i++ ) { 
      for( int j =0; j < integral_images[i].size() ; j++ )
	delete (integral_images[i].iiVector.at(j)); 
    }
    delete[] integral_images;
	  
    for( int i = 0; i < nclasses; i++ ) { 
      for( int j = 0; j < test_integral_images[i].size(); j++ ) 
	delete (test_integral_images[i].iiVector.at(j)); 
    }
	  
    delete[] test_integral_images; 

  }      
};
  
/** random x index 
 * @param config the config class instance 
 */
int rand_row_index( ForestConfigClass &config )
{ 
  return rand()%config.image_info_instance.get_height(); 
}
/** random y index
 * @param config the config class instance 
 */
int rand_col_index( ForestConfigClass &config )
{ 
  return rand()%config.image_info_instance.get_width(); 
}

/**
 * Test candidate for the forest 
 */ 
class ForestTest { 

  
 public:
  /** all images that get assigned on left brnach on test candidate */ 
  VectorOfIntegralImages *left_branch; 
  /** all images that get assigned to rifght branch on test candidate */ 
  VectorOfIntegralImages *right_branch; 
  /** the threshold theta1, */
  double theta1; 
  /** theta2 */
  double  theta2;
  /** top left coordinates of randomrectangle sampled */ 
  int xmain; 
  /** top left coordinates of randomrectangle sampled */    
  int ymain;
    
  /** width of the sampled random rectangle */ 
  int xdash; 
  /** height of the sampled random rectangle */     
  int ydash;

  /** haar featuree considered */ 
  int haar_feature;
	
	
 private:
	
  /** entropy value of the test candidate */ 
  double __entropy_value; // the value of the entropy between the left and right branches.
    
  /** 
   * calculation of entropy using class histogram and derived probabiliities 
   */ 
  double entropy_calculation(ForestConfigClass& config)
  {
    double finalProbabilityLeft = 0.0;
    double finalProbabilityRight = 0.0;
    double probabilityLeft = 0.0, probabilityRight = 0.0;
    int numeratorLeft = 0, numeratorRight = 0;
    int denominatorLeft = 0;
    int denominatorRight = 0;
    int i=0;
	  
    for(i=0;i<config.get_nclasses();i++)
      {
	denominatorLeft += left_branch[i].iiVector.size()+1;
	denominatorRight += right_branch[i].iiVector.size()+1;
      }
	  
    for(i=0;i<config.get_nclasses();i++)
      {
	numeratorLeft = left_branch[i].iiVector.size()+1; 
	numeratorRight = right_branch[i].iiVector.size()+1;
	probabilityLeft = (double) numeratorLeft/denominatorLeft;
	probabilityRight = (double) numeratorRight/denominatorLeft;
		
	finalProbabilityLeft += probabilityLeft*log2(probabilityLeft);
	finalProbabilityRight += probabilityRight*log2(probabilityRight);
      }
    finalProbabilityLeft *=-1.0;
    finalProbabilityRight *=-1.0;
	  
    if(finalProbabilityLeft > finalProbabilityRight) // en.wikipedia.org/wiki/Shannon_entropy
      return (__entropy_value = finalProbabilityLeft);
    else
      return (__entropy_value = finalProbabilityRight);
	  
  }
	
 public:
  /** 
   * get entropy for split by test candidate
   * @param config ConfigClass Instance 
   */ 
  double get_entropy_value(ForestConfigClass &config) { return (__entropy_value=entropy_calculation(config));}


  /** 
   * a constructor 
   * @param copy_test created to copy a chosen test candidate and store it at the node
   * @param config ConfigClass Instance 
   */

  ForestTest(ForestTest* copy_test, ForestConfigClass& config)
    {
	  
      left_branch = new VectorOfIntegralImages[ config.get_nclasses() ];
      right_branch = new VectorOfIntegralImages[ config.get_nclasses() ]; 
      xdash = ydash = xmain = ymain = haar_feature = 0;
      theta1 = theta2 = 0.0; 

      for(int i=0;i<config.get_nclasses();i++){
		
	for(unsigned int j=0; j<copy_test->left_branch[i].iiVector.size(); j++)
	  left_branch[i].iiVector.push_back( copy_test->left_branch[i].iiVector.at(j) );
		
		
	for(unsigned int j=0; j<copy_test->right_branch[i].iiVector.size(); j++)
	  right_branch[i].iiVector.push_back( copy_test->right_branch[i].iiVector.at(j) );
      }
	  
      theta1 = copy_test->theta1;
      theta2 = copy_test->theta2;
      xdash = copy_test->xdash;
      ydash = copy_test->ydash;
      xmain = copy_test->xmain;
      ymain = copy_test->ymain;
	  
      __entropy_value = copy_test->get_entropy_value(config);
      haar_feature = copy_test->haar_feature;
    }

  /** 
   * a constructor that works form the configClass object
   * @param config ConfigClass Instance
   */ 
 ForestTest(ForestConfigClass& config):/*left_branch(0), right_branch(0), */theta1(0.0), theta2(0.0), xmain(0), 
    ymain(0), xdash(0), ydash(0), haar_feature(0) {
    left_branch = new VectorOfIntegralImages[ config.get_nclasses() ];
    right_branch = new VectorOfIntegralImages[ config.get_nclasses() ];
  }

  /** 
   * a constructor that takes in the top left coordinates of the random rectangle 
   * @param x top left x coord
   * @param y top left y coord 
   * @param config ConfigClass Instance 
   */ 
 ForestTest(int x, int y, ForestConfigClass& config): theta1(0.0), theta2(0.0), xmain(x),ymain(y), xdash(0), ydash(0),
    haar_feature(0)	 {
	  
    left_branch = new VectorOfIntegralImages[ config.get_nclasses() ];
    right_branch = new VectorOfIntegralImages[ config.get_nclasses() ];
	  
    theta1 =random_booster();
    theta2 =random_booster();
	  
    while( (xdash = random_rectangle()) < RECTANGULAR_MIN_FEATURE_SIZE ) 
      xdash = random_rectangle();
	  
    while( (ydash = random_rectangle()) < RECTANGULAR_MIN_FEATURE_SIZE )
      ydash = random_rectangle();
	  
	  
    double temp;
    if(theta1>theta2) { 
      temp = theta1;
      theta1 = theta2;
      theta2 = temp;
    }
	  
    __entropy_value = -1.0;
  }


  /** Does the image pass the test - as defined by a region and threshold? 
   * @param integral_image - the integral image of the test image
   * @param xmain top left x coordinate 
   * @param ymain top left y coordinate 
   * @param xdash width 
   * @param ydash height 
   * @param theta1 first threshold 
   * @param theta2 second threshold 
   * @param height height of the training images 
   * @param width width of the training images 
   * @param haar_feature haar feature taken into consideration 
   */

  bool image_pass(int* integral_image, int xmain, int ymain, int xdash, 
		 int ydash, double theta1, double theta2, int height, int width, int haar_feature){
	  
    return feature_pass(
		       get_rectangular_integral_image_feature(
							  integral_image, xmain, ymain, 
							  xdash, ydash, height, width, 
							  haar_feature), 
		       theta1, theta2, 
		       NOS_THRESHOLDS);
  }

  /** Run tests on the integral images and return final entropy of the division. 
   * @param integral_images vector of II 
   * @param haar_feature haar feature consdiered
   * @param config ConfigClass Instance
   */
  double run_tests_on_integral_images(VectorOfIntegralImages *integral_images, 
				  int haar_feature, ForestConfigClass& config){
	  
	  
    for(int i=0;i<config.get_nclasses();i++){
      left_branch[i].iiVector.clear();
      right_branch[i].iiVector.clear();
    }
	  
    this->haar_feature = haar_feature;
	  
    int *integralImage;
	  
    for(int i=0;i<config.get_nclasses();i++)
      for(unsigned int j=0;j<integral_images[i].iiVector.size();j++){
	integralImage = (integral_images[i].iiVector.at(j));
		  
	if(image_pass((integralImage), xmain, ymain, xdash, ydash, theta1, 
		     theta2, config.image_info_instance.get_height(), 
		     config.image_info_instance.get_width(), haar_feature))
	  left_branch[i].iiVector.push_back(integral_images[i].iiVector.at(j));
	else
	  right_branch[i].iiVector.push_back(integral_images[i].iiVector.at(j));
      }
	  
    integralImage = 0; 					
	  
	  
    return get_entropy_value(config);
  }
	
	
  /** destrcutor for test */ 	
  ~ForestTest()
    {
	  
      delete[] left_branch;
      delete[] right_branch;

    }
	
};
  
/**
 * Builds a random tree by instantiating L number of tests
 */ 
class Tree {
 private:
  /** level of the tree */ 
  int __tree_level; // was initially static int TreeLevel
  /** index of the forest */ 
  int __forest_index; // Useful to control the forest; has a value of -1 if the tree is not Root
  /** entropy of the parent node for comparisons */
  double __parent_entropy;
  /** index of the current node */ 
  int __node_index;
  /** index of the parent node */ 
  int __parent_node_index;
  /** level of the node in the main tree */
  int __level;
  
  

	
 public:
  /** a global node counter */
  int global_node_index;
  /** vector of integral images for the tree */ 
  VectorOfIntegralImages *the_integral_images;
  /** best candidate chosen */ 
  ForestTest* best_test;
  /** link the left child */
  Tree* left_tree;
  /** link to the right child */
  Tree* right_tree;
  /** how difficult is to split the images at the node? */ 
  int max_try; 
	
  /** get the level of the tree */ 
  int get_tree_level() const { return __tree_level; }
	
  /** is the node allowed to grow further? */
  bool is_node_allowed() const { return ( max_try > MAX_TRY_ALLOWED )?true:false; }
	
  /** are the images at the node difficult to divide here? */ 
  void node_try() { max_try++; }
	
  /** what is the index of the current random tree */
  int get_forest_index() const { return __forest_index; }
	
  /** get a unique number to label the nodes of the random tree */ 
  int get_unique_number() const 
  {
    return __node_index; 
  }
	
  /** increment the level of the tree growth */
  int increment_level() { return __tree_level++; } 
	
  /** destructor */
  ~Tree()
    {
	  
      delete best_test;
      delete left_tree; 
      delete right_tree; 
      delete[] the_integral_images; 
      
    }


  /** Tree constructor for all nodes other than the "main" root node 
   * @param integral_images vector of integral images at the node 
   * @param pE entropy of the parent node 
   * @param level_of_tree level of the node in the main tree 
   * @param sNodeIndex node index of the current node 
   * @param sParentNodeIndex node index of the parent node
   * @param config ConfigClass Instance 
   */
 Tree(VectorOfIntegralImages* integral_images, double pE, int level_of_tree, 
      int sNodeIndex, int sParentNodeIndex, ForestConfigClass &config): 
  __forest_index(-1),__parent_entropy(pE), __node_index(sNodeIndex), 
    __parent_node_index(sParentNodeIndex), __level(level_of_tree), 
    best_test(0), left_tree(0), right_tree(0), max_try(0)  {
	  
    the_integral_images = new VectorOfIntegralImages[config.get_nclasses()];
	  

    for(int i=0;i<config.get_nclasses();i++)
      the_integral_images[i].iiVector.assign(integral_images[i].iiVector.begin(), 
					   integral_images[i].iiVector.end());
	 
  }



  /** Tree constructor for "only" the root node 
   * @param config ConfigClass Instance
   * @param globalNodeIndex global counter to monitor node indices
   * @param forest_index_of_root number of the random tree in the forest 
   */ 
 Tree( ForestConfigClass &config, int &globalNodeIndex, 
       int forest_index_of_root): 
  __forest_index(forest_index_of_root), __parent_entropy(200.0), 
    __node_index(globalNodeIndex++), __parent_node_index(-1), 
    __level(0), best_test(0), left_tree(0), 
    right_tree(0), max_try(0)
    {
	  
      the_integral_images = new VectorOfIntegralImages[ config.get_nclasses() ];
	 
	  
      for( int i =0; i < config.get_nclasses(); i++ )
	the_integral_images[i].iiVector.assign( 
					     config.integral_images[i].iiVector.begin(), 
					     config.integral_images[i].iiVector.end() ); 
      //		 } 
	  
	  
      __level = increment_level();
	  
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
	  temp_test = new ForestTest(rand_row_index(config),rand_col_index(config), config); 
	  testsMade.push_back( temp_test );
	} 
	  
      /* I am going to run the tests with each of the Haar features */
      for(haarIndex = 0; haarIndex < HAARFEATURES; haarIndex++ )
	{
	  for(i=0;i<NITERATIONS;i++)
	    entropies[i] = testsMade.at(i)->run_tests_on_integral_images( the_integral_images, haarIndex, config );
		
	  finalIndex = find_min( entropies, NITERATIONS );
	  entropiesHaar[haarIndex] = entropies[finalIndex];
	  testIndex[haarIndex] = finalIndex;
	}	
	  
	  
      /* 
	 find min of the best entropies for each of the haar features - 
	 so in essence, haarIndex has the best Haar feature for the 
	 200 tests considered
      */
      haarIndex = find_min(entropiesHaar, HAARFEATURES);
      /*
	re-use final index to store the index of the best test 
	( a test is instantiated by xmain,YMain,xdash, ydash,
	theta1, theta2 & a HAAR feature found out here. So, we keep
	track of the other things 
      */
      finalIndex = testIndex[haarIndex];
      /* 
	 since the allocation of images will be different in the
	 given testsMade vector (it will be of the last test run) , we run it again 
      */  
      testsMade.at(finalIndex)->run_tests_on_integral_images(the_integral_images, haarIndex, config);
      best_test = new ForestTest(testsMade.at(finalIndex), config);
	  
      for( unsigned int k = 0; k < testsMade.size(); k++ ) 
	{ 
	  temp_test = testsMade.at( k ); 
	  delete temp_test; 
	}         
	  
      temp = 0; 
      testsMade.clear();
	  
      if(WR_DOT_FILE) {		   /* graphviz output of the root being divided */
	strcpy(bins," ");
	for(i=0;i<config.get_nclasses()-1;i++){
	  sprintf(buffer1,"%zu", the_integral_images[i].iiVector.size());		  
	  strcat(bins, buffer1);
	  strcat(bins,",");
	}
	sprintf(buffer1,"%zu",the_integral_images[config.get_nclasses()-1].iiVector.size());		  
	strcat(bins, buffer1);
	sprintf(buffer1, "%f",best_test->get_entropy_value(config));
	sizeOfSprintf = sprintf(buffer,
				"node%d[label = \"<f0> %d| <f1> %s |<f2> %s \"];",
				__node_index,__node_index,bins,
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
      i = increment_level(); 
      int global_node_index = 0; 
	  
      /* left tree obtains the left branch of the best split. */
      left_tree = new Tree(best_test->left_branch, best_test->get_entropy_value(config), 
			   i, global_node_index++ , __node_index, config );
	  
      if( WR_DOT_FILE ) { 
		
	/* grapbviz for left tree */
	sizeOfSprintf = sprintf(buffer,"\"node%d\":f0 -> \"node%d\":f1;",__node_index,left_tree->__node_index);
	graphviz.write(buffer, sizeOfSprintf);
      }
	  
      /* right tree obtains the right branch of the best split */
      right_tree = new Tree(best_test->right_branch, best_test->get_entropy_value(config), 
			   i, global_node_index++, __node_index, config );
	  
      if( WR_DOT_FILE ) { 
		
	/* graphviz for the right tree */
	sizeOfSprintf = sprintf(buffer,"\"node%d\":f2 -> \"node%d\":f1;",__node_index,right_tree->__node_index);
	graphviz.write(buffer, sizeOfSprintf);
      }	   
	  
      /* 
	 clear vector of the children to start splitting. 
	 (in case of garbage values) 
      */
      childrenToTest.clear();
      /* put the left and right trees on the stack */
      childrenToTest.push_back(left_tree);
      childrenToTest.push_back(right_tree);
      /* clear another vector (clear out garbage) */
      tempChildrenToTest.clear();
	  
	  
      while(global_node_index<NODECOUNT && childrenToTest.size()!=0){		  
	/* we want only NODECOUNT to be total number of nodes created */
		
		
		
	for(i=0;i<NITERATIONS;i++) { 
	  temp_test = new ForestTest(rand_row_index(config),rand_col_index(config), config); 
	  testsMade.push_back( temp_test );
	}
		
		
	    for(int j=0;j<(int)childrenToTest.size();j++){
		  
	      /* run tests for the images for the jth child */
	      for(haarIndex = 0; haarIndex<HAARFEATURES; haarIndex++){
		for( i=0;i < NITERATIONS; i++)
		  entropies[i] = testsMade.at(i)->
		    run_tests_on_integral_images(childrenToTest.at(j)->the_integral_images, 
						 haarIndex, config);
			
			
		finalIndex = find_min(entropies, NITERATIONS);
		entropiesHaar[haarIndex] = entropies[finalIndex];
		testIndex[haarIndex] = finalIndex;
	      }
		  
		  
	      haarIndex = find_min(entropiesHaar, HAARFEATURES);
	      finalIndex = testIndex[haarIndex];
		  
	      testsMade.at(finalIndex)->
		run_tests_on_integral_images(childrenToTest.at(j)->the_integral_images, 
					 haarIndex, config);
		  
	      childrenToTest.at(j)->best_test = new ForestTest(testsMade.at(finalIndex), config);
		  
	      /* graphviz instructions - create the node (label it)*/
	      if( WR_DOT_FILE )
		strcpy(bins," ");
		  
	      for(i=0;i<config.get_nclasses()-1;i++){
			
		if( WR_DOT_FILE )
		  sprintf(buffer1,"%zu",childrenToTest.at(j)->the_integral_images[i].iiVector.size());
		if( WR_DOT_FILE ) { 
		  strcat(bins, buffer1);
		  strcat(bins,",");
		}
	      }
		  
	 
		  
	      if(WR_DOT_FILE) { 
		sprintf(buffer1,"%zu",childrenToTest.at(j)->the_integral_images[config.get_nclasses()-1].iiVector.size());
		strcat(bins, buffer1);
		sprintf(buffer1, "%f",childrenToTest.at(j)->best_test->get_entropy_value(config));
		sizeOfSprintf = sprintf(buffer,
					"node%d[label = \"<f0> %d| <f1> %s |<f2> %s \"];", 
					childrenToTest.at(j)->__node_index, 
					childrenToTest.at(j)->__node_index,bins,buffer1);
		graphviz.write(buffer, sizeOfSprintf);
			
	      }
		  
	      //		if(!checkGrowth){
		  
	      bool left_branchIsEmpty = true;
	      bool right_branchIsEmpty = true;
		  
	      for( int k = 0 ; k < config.get_nclasses() ; k++ )
		if( testsMade.at( finalIndex )->left_branch[ k ].size() != 0 ) {
		  left_branchIsEmpty = false; 
		}
		  
		  
	      for( int k = 0 ; k < config.get_nclasses() ; k++ ) 
		if( testsMade.at( finalIndex )->right_branch[ k ].size() != 0 ) { 
		  right_branchIsEmpty = false; 
		}
		  
		  
		  
	      if( !left_branchIsEmpty )
		childrenToTest.at(j)->left_tree = new Tree(testsMade.at(finalIndex)->left_branch, 
							  entropiesHaar[haarIndex], temp, 
							  global_node_index++, 
							  childrenToTest.at(j)->__node_index, config);
		  
	      if( WR_DOT_FILE && !left_branchIsEmpty ) 
		{ 
		  /* establish graphviz connection between parent & this new child */
		  sizeOfSprintf = sprintf(buffer,"\"node%d\":f0 -> \"node%d\":f1;",
					  childrenToTest.at(j)->__node_index, 
					  childrenToTest.at(j)->left_tree->__node_index);
		  graphviz.write(buffer, sizeOfSprintf);
		}
		  
		  
	      if( !right_branchIsEmpty )
		childrenToTest.at(j)->right_tree = new Tree(testsMade.at(finalIndex)->right_branch, 
							   entropiesHaar[haarIndex], temp, 
							   global_node_index++, 
							   childrenToTest.at(j)->__node_index, config);
		  
	      if( WR_DOT_FILE && !right_branchIsEmpty ) 
		{ 
		  /* establish graphviz connection between parent & new child */
		  sizeOfSprintf = sprintf(buffer,"\"node%d\":f2 -> \"node%d\":f1;",
					  childrenToTest.at(j)->__node_index,
					  childrenToTest.at(j)->right_tree->__node_index);
			
		  graphviz.write(buffer, sizeOfSprintf);
			
		}
		  
		  
	      /* store the newly created trees for tests in the next iteration */
		  
	      if( !left_branchIsEmpty )
		tempChildrenToTest.push_back(childrenToTest.at(j)->left_tree);
	      if( !right_branchIsEmpty )
		tempChildrenToTest.push_back(childrenToTest.at(j)->right_tree);
	
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
  void grow_tree_from_images( char* train_images, int nclasses, Tree* &root, int &train_height, int &train_width, ForestConfigClass& config  )
  {
    // Define a ConfigClass
	
    config.set_subwindow_range( SUBWINDOW_RANGE ); 
    config.set_train_images( train_images ); 

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

    mainDirPath = config.get_train_images(); 
	
    for(i = 0;i<config.get_nclasses();i++){
	  
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
			
	  config.image_info_instance.set_height( tempImage->height );
	  config.image_info_instance.set_width( tempImage->width );
			
	  train_height = tempImage->height; 
	  train_width = tempImage->width; 
			
	  tempII = new int[ ( tempImage->height ) * ( tempImage->width ) ];
			
	  calculate_integral_image( tempImage, tempII );
			
	  config.integral_images[i].iiVector.push_back( tempII );
			
	  if( !setBorders ) { 
			  
	    config.integral_images[i].set_height( tempImage->height ); 
	    config.integral_images[i].set_width( tempImage->width ); 
	    setBorders = true; 
	  }
			
			
	  cvReleaseImage( &tempImage ); 
	}  
		  
	flagNotImage = false;
      } 
    } 	  
	  
    int global_node_index = 0;
    int number_of_trees_grown = 0;
    root = new Tree( config, global_node_index, number_of_trees_grown++ ); 

		
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
  vector<Tree*> __forest; 
  /** number of identiites */ 
  int __nclasses; 
  /** the size of the forest */
  int __forest_size; 
  /** the height of the training images */ 
  int __train_height; 
  /** the width of the trainign images */ 
  int __train_width; 

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
    
      __nclasses = nclasses; 
      __forest_size = forestSize; 

    
      for(int i = 0; i < __forest_size ; i++ ) 
	{ 

   
	  int global_node_index = 0;
	  int __number_of_trees = 0;
	  if( i == 0 )
	    {   
	      Tree* tempTree; 
	      grow_tree_from_images( train_images, nclasses, tempTree, train_height, train_width, config ); 
	      __train_height = train_height; 
	      __train_width = train_width; 
	      __forest.push_back( tempTree ); 
	    }
    
	  else 
	    {
	      global_node_index = 0 ;
	      __number_of_trees = i; 
	      __forest.push_back( new Tree( config, global_node_index, __number_of_trees ) ); 
	    }
    
    
	}
   
      
    }
   

  /** get the forest size */ 
  int get_size() const 
  { 
    return __forest.size(); 
  }
    
  /** get the numner of obeject classes/identities */ 
  int get_nclasses() const 
  { 
    return __nclasses; 
  }
    
  /** get the height of the training samples - for rescaling test images */ 
  int get_train_height() const 
  { 
    return __train_height; 
  }
     
  /** get the width of the training samples - for rescaling test images */ 
  int get_train_width() const 
  { 
    return __train_width;
  }
       


  /** get the random tree of index i from the forest 
   * @param i get the tree of the index in the forest 
   */ 
  Tree* get_tree( int i ) 
  { 
  
    return __forest.at(i);
  }


  /** destrcutor */    
  ~ForestClass() 
    {

      for( unsigned int i = 0; i < __forest.size(); i++ ) 
	delete __forest.at(i); 

    } 
};
  	


/** get classification results for any random tree 
 * @param aTree the random tree
 * @param ii the integral image
 * @param train_height height of the training images
 * @param train_width widht of the trainign images
 * @param nclasses number of indentities 
 */ 
int run_classification_for_arbitrary_tree( Tree* aTree, int* &ii, int train_height, int train_width, int nclasses )
{
	
  Tree *root = aTree, *prevTree = aTree; 
  bool child_pass; 
  ForestTest* root_test = 0;
	
 	
  while( root!= 0 )
    {
      if( root->left_tree == 0 || root->right_tree == 0 )
	{
	  if( root->the_integral_images == 0 ) 
	    {
	      root = prevTree;
	      break;
	    }
	  else
	    break;
	}
	  
      root_test = root->best_test; 
	  
      if( root_test == 0 )
	{
	  root = prevTree; 
	  break; 
	}
	  
      child_pass = root_test->image_pass( ii, root_test->xmain, root_test->ymain, root_test->xdash, root_test->ydash, root_test->theta1, root_test->theta2, train_height, train_width, root_test->haar_feature ); 
	  
      prevTree = root; 
	  
      if( child_pass ) 
	root = root->left_tree; 
      else
	root = root->right_tree; 
	  
    }
	
  int max = -1; 
  int index = -2; 
  int total_size = 0; 
	
  for( int i = 0; i < nclasses; i++ ) 
    {
	  
      if( (int)(root->the_integral_images[i].iiVector.size()) > max ) 
	{
	  max  = root->the_integral_images[i].iiVector.size();
	  index = i; 
		
	  total_size += root->the_integral_images[i].iiVector.size(); 
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
int return_class_label_for_IplImage( Tree *aTree, IplImage *imageToClassify, int train_height, int train_width, int nclasses )
{
	
  int* ii = new int[ imageToClassify->height * imageToClassify->width ]; 
  calculate_integral_image( imageToClassify, ii );
	
  return run_classification_for_arbitrary_tree( aTree, ii, train_height, train_width, nclasses  ) ; 
	

}

  
/** for a given forest class instance return the classification label. 
 * @param forestClassInstance the forest class instance
 * @param inputImage the input image
 */ 
int get_class_label_from_forest( ForestClass* forestClassInstance , IplImage* inputImage )
{
    
  int recognition_histogram[ forestClassInstance->get_size() ];
    
  for( int i = 0; i < forestClassInstance->get_size(); i++ ) 
    recognition_histogram[i] = return_class_label_for_IplImage( forestClassInstance->get_tree(i), inputImage, forestClassInstance->get_train_height(), forestClassInstance->get_train_width(), forestClassInstance->get_nclasses() );  
      
  int votes[ forestClassInstance->get_nclasses() ]; 
    
  for( int i = 0; i < forestClassInstance->get_nclasses(); i++ ) 
    votes[i] = 0;
        
  for( int i = 0; i < forestClassInstance->get_size(); i++ ) 
    votes[ recognition_histogram[i] ]++; 
        
  int max_identity = -1; 
  int index_max_identity = -1; 
	  
  for( int i = 0; i < forestClassInstance->get_nclasses() ; i++ ) 
    if( votes[i] > max_identity ) 
      {
	max_identity = votes[i]; 
	index_max_identity = i; 
      }
	  

  return index_max_identity; 
    
}
	
	

#endif
