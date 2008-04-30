#include "Parameters.hh"
#include "Auxillary.hh"
#include <utils/logging/liblogger.h>

#ifndef __forest__hh__ 
#define __forest__hh__

/***************************************************************************
 *  Forest.hh - Header file for object recognition with random forests: All Forest related data structures and operations are present here
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



extern double randomBooster();

namespace Forest {
  
  using UserDef::VectorOfIntegralImages;
  using UserDef::getRectangularIntegralImagefeature;
  using Auxillary::randomRectangle; 
  using Auxillary::featurePass;
  using UserDef::ConfigClass;
  using Auxillary::randRowIndex;
  using Auxillary::randColIndex;
  using Auxillary::findMin;
  
  /** 
   * The test data structure is used to test image features. 
   * It intializes all parameters necessary to carry out the test.  
   */ 
  class Test{
	
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

    /** haar feature considered */ 
	int haarFeature;
	
	
  private:
	
    /** entropy value of the test candidate */ 
	double entropyValue; // the value of the entropy between the left and right branches.
    
    /** 
     * calculation of entropy using class histogram and derived probabiliities 
     */ 
	double entropyCalculation(UserDef::ConfigClass& config)
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
     * \param config ConfigClass Instance 
     */ 
	double getEntropyValue(ConfigClass &config) { return (entropyValue=entropyCalculation(config));}

    /** 
     * a constructor 
     * \param copyTest created to copy a chosen test candidate and store it at the node
     * \param config ConfigClass Instance 
     */
    Test(Test* copyTest, UserDef::ConfigClass& config)
    {
	  
	  leftBranch = new VectorOfIntegralImages[ config.getNclasses() ];
	  rightBranch = new VectorOfIntegralImages[ config.getNclasses() ]; 
	  
	  //	  leftBranch = rightBranch = 0; 
	  xdash = ydash = xMain = yMain = haarFeature = 0;
	  theta1 = theta2 = 0.0; 
	  
	  
	  //		 leftBranch = new VectorOfIntegralImages[config.getNclasses()];
	  //		 rightBranch = new VectorOfIntegralImages[config.getNclasses()];
	  
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
     * \param config ConfigClass Instance
     */ 
	Test(UserDef::ConfigClass& config):/*leftBranch(0), rightBranch(0), */theta1(0.0), theta2(0.0), xMain(0), 
	yMain(0), xdash(0), ydash(0), haarFeature(0) {
	  leftBranch = new VectorOfIntegralImages[ config.getNclasses() ];
	  rightBranch = new VectorOfIntegralImages[ config.getNclasses() ];
	  
	}
	
    
    /** 
     * a constructor that takes in the top left coordinates of the random rectangle 
     * \param x top left x coord
     * \param y top left y coord 
     * \param config ConfigClass Instance 
     */ 
    Test(int x, int y, UserDef::ConfigClass& config):/*leftBranch(0),rightBranch(0),*/ theta1(0.0), theta2(0.0), xMain(x),yMain(y), xdash(0), ydash(0),
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
	  //		 leftBranch = new VectorOfIntegralImages[config.getNclasses()];
	  //		 rightBranch = new VectorOfIntegralImages[config.getNclasses()];
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
     * \param integralImages vector of II 
     * \param haarFeature haar feature consdiered
     * \param config ConfigClass Instance
    */
	double runTestsOnIntegralImages(VectorOfIntegralImages *integralImages, 
									int haarFeature, ConfigClass& config){
	  
	  // 1. Clear the contents of the left and right branch
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
	  
	  // getEntropyValue() encloses the entropyCalculate() module
	  return getEntropyValue(config);
	}
	
    /** destrcutor for test */ 	
	~Test()
	{
	  
	  
	  
//	  for( int i = 0; i < config.getNclasses(); i++ ) { 
	//	for( int j = 0; j < leftBranch[i].size(); j++ ) 
	  

	  delete[] leftBranch;
	  delete[] rightBranch;
		  //		  leftBranch[i].iiVector.at( j ) = 0; 
		
	//	leftBranch[i].clear(); 
	 // }
	  
	 // for( int i = 0; i < config.getNclasses(); i++ ) 
	  //{ 
	  /*
		for( int j = 0; j < rightBranch[i].size(); j++ )
		  rightBranch[i].iiVector.at( j ) = 0; 
		
		rightBranch[i].clear(); 
	  }
	 */ 
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
	Test* bestTest;
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
	  /*
	  for( int i = 0; i < config.getNclasses(); i++ ) {
		for( int j = 0; j < theIntegralImages[i].size(); j++ )
		  theIntegralImages[i].iiVector.at( j ) = 0;	
		
		
		theIntegralImages[i].clear();	
	  }*/
	  
	  //delete[] theIntegralImages;
	  // }
	  
	  
	  //	if(bestTest!=0)
	  //	delete bestTest;
	  
	  
	  //		 if(leftTree!=0)
	  //	delete leftTree;
	  
	  
	  //	if(rightTree!=0)
	  //	delete rightTree;
	  
	}
	
    /** Tree constructor for all nodes other than the "main" root node 
     * \param integralImages vector of integral images at the node 
     * \param pE entropy of the parent node 
     * \param levelOfTree level of the node in the main tree 
     * \param sNodeIndex node index of the current node 
     * \param sParentNodeIndex node index of the parent node
     * \param config ConfigClass Instance 
     */
	Tree(VectorOfIntegralImages* integralImages, double pE, int levelOfTree, 
		 int sNodeIndex, int sParentNodeIndex, ConfigClass &config): 
	ForestIndex(-1),parentEntropy_(pE),  nodeIndex_(sNodeIndex), 
	parentNodeIndex_(sParentNodeIndex), level_(levelOfTree), 
	bestTest(0), leftTree(0), rightTree(0), max_try(0)  {
	  
	  theIntegralImages = new VectorOfIntegralImages[config.getNclasses()];
	  
	  //		 if(integralImages ==0)
	  //	theIntegralImages = 0;
	  
	  //		 else {
	  
	  //	theIntegralImages = new VectorOfIntegralImages[config.getNclasses()];
	  
	  for(int i=0;i<config.getNclasses();i++)
		theIntegralImages[i].iiVector.assign(integralImages[i].iiVector.begin(), 
											 integralImages[i].iiVector.end());
	  //		 }
	}
	
	
    /** Tree constructor for "only" the root node 
     * \param config ConfigClass Instance
     * \param globalNodeIndex global counter to monitor node indices
     * \param ForestIndexOfRoot number of the random tree in the forest 
     */ 
	Tree( UserDef::ConfigClass &config, int &globalNodeIndex, 
		 int ForestIndexOfRoot): 
	ForestIndex(ForestIndexOfRoot), parentEntropy_(200.0), 
	nodeIndex_(globalNodeIndex++), parentNodeIndex_(-1), 
	level_(0), bestTest(0), leftTree(0), 
	rightTree(0), max_try(0)
	{
	  
	  theIntegralImages = new VectorOfIntegralImages[ config.getNclasses() ];
	  //Store the vector of pointers to integral images
	  //		 if( integralImages==0 ) 
	  //	theIntegralImages = 0; 
	  
	  //		 else {
	  
	  //		theIntegralImages = new VectorOfIntegralImages[config.getNclasses()];
	  
	  
	  // VectorOfIntegralImages integralImages[ config.getNclasses() ];
	  
	  //		 // Emulate integralImages 
	  //		 for(int i = 0 ; i < config.getNclasses() ; i++ ) 
	  //	for( int j = 0 ; j < config.integralImages[i].size() ; j++ )
	  //	  integralImages[i].iiVector.push_back( &(config.integralImages[i].iiVector.at(j)) ); 
	  
	  
	  
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
	  Test* temp_test = 0; 
	  
	  
	  if( WR_DOT_FILE ) 
	  {  // if GRAPHVIZ output is necessary 
		graphviz.open(treedot.c_str());
		buffer = new char[ 300 ];
		bins = new char[ 200 ];
		buffer1 = new char[ 50 ]; 
		
		int sizeOfSprintf = sprintf(buffer,"digraph g { node [shape = record,height=.1];"); 
		graphviz.write(buffer, sizeOfSprintf);
	  }
	  
	  vector< Test* >  testsMade; // the 200 or 'N' number of tests created
	  int finalIndex = -1;  // the index of the best test: greatest entropy value
	  double* entropies = 0 ;
	  entropies = new double[NITERATIONS];
	  
	  int i=0, temp=0;
	  int haarIndex = 0;
	  //	  int numberContaining = 0;
	  //	  bool checkGrowth; // dont split for (0,0,0,0,0.....0,0,0,0) etc 
	  double entropiesHaar[HAARFEATURES];
	  int testIndex[HAARFEATURES];
	  
	  for(i =0;i<NITERATIONS;i++)  // make tests
	  { 
		temp_test = new Test(randRowIndex(config),randColIndex(config), config); 
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
	  bestTest = new Test(testsMade.at(finalIndex), config);
	  
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
		  temp_test = new Test(randRowIndex(config),randColIndex(config), config); 
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
		  
		  childrenToTest.at(j)->bestTest = new Test(testsMade.at(finalIndex), config);
		  
		  
		  
		  //	  if(childrenToTest.at(j)->bestTest->getEntropyValue()!=childrenToTest.at(j)->parentEntropy_){ 
		  
		  //	  checkGrowth = false; 
		  
		  
		  
		  /*
		   *********************
		   * Section deals with the conditions that prevent a node that doesnt split into a node consistently
		   ***********
		   */
		  
		  
		  //	  if( childrenToTest.at(j)->bestTest->getEntropyValue() == childrenToTest.at(j)->parentEntropy_ ) { 
		  //		/*
		  //		 * no division has taken place.
		  //		 * To resolve we let it have MAX_TRY_ALLOWED number of tries. 
		  //		 * If nothing happens till then, we shall leave it as it is.
		  //		 */
		  
		  //		delete childrenToTest.at(j)->bestTest;
		  
		  //		childrenToTest.at(j)->bestTest = 0;
		  //		childrenToTest.at(j)->leftTree = 0;
		  //		childrenToTest.at(j)->rightTree = 0; 
		  
		  
		  //		if( childrenToTest.at(j)->isNodeAllowed() ) { 
		  
		  //		  // Push it in the new stack; 
		  //		  tempChildrenToTest.push_back( childrenToTest.at(j) ); 
		  //		  childrenToTest.at(j)->nodeTry(); 
		  //		  continue; 
		  
		  //		}
		  
		  //		else 
		  //		  continue;
		  
		  //	  }
		  
		  /*
		   * END of SEC
		   */	 
		  
		  /***********
		   * If there is an improvement, then do the splits! 
		   ************
		   */
		  
		  
		  //		/* if there is an improvment, then ..... */
		  
		  //		/* toggle the values of checkGrowth: see below */
		  //		checkGrowth = true; 
		  
		  //		/* if at the end, numberContaining == 1 then dont split anymore */
		  //		//		numberContaining = 0; 
		  
		  
		  /* graphviz instructions - create the node (label it)*/
		  if( WR_DOT_FILE )
			strcpy(bins," ");
		  
		  for(i=0;i<config.getNclasses()-1;i++){
			
			if( WR_DOT_FILE )
			  sprintf(buffer1,"%zu",childrenToTest.at(j)->theIntegralImages[i].iiVector.size());
			//		if(childrenToTest.at(j)->theIntegralImages[i].iiVector.size()!=0) {
			//		  checkGrowth = false; 
			//		  numberContaining++;
			//		}
			if( WR_DOT_FILE ) { 
			  strcat(bins, buffer1);
			  strcat(bins,",");
			}
		  }
		  
		  
		  // If the following section is commented then, 
		  // the tree will continue to divide even if there is only one bin 
		  // which is full ( and all others are empty).
		  //		if(numberContaining == 1)
		  //checkGrowth = true;
		  
		  
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
		  //		}
		  
		  
		  //	 else {
		  
		  //		delete childrenToTest.at( j )->bestTest; 
		  //		childrenToTest.at(j)->bestTest = 0; 
		  
		  //		if( WR_DOT_FILE ) { 
		  //		  strcpy(bins," ");
		  //		  for(i=0;i<config.getNclasses()-1;i++){
		  //		sprintf(buffer1,"%d",childrenToTest.at(j)->theIntegralImages[i].iiVector.size());
		  //		strcat(bins, buffer1);
		  //		strcat(bins,",");
		  //		  }
		  
		  //		  sprintf(buffer1,"%d",childrenToTest.at(j)->theIntegralImages[config.getNclasses()-1].iiVector.size());
		  //		  strcat(bins, buffer1);
		  //		  sprintf(buffer1, "%f",childrenToTest.at(j)->bestTest->getEntropyValue());
		  //		  sizeOfSprintf = sprintf(buffer,
		  //					  "node%d[label = \"<f0> %d| <f1> %s |<f2> %s \"];",
		  //					  childrenToTest.at(j)->nodeIndex_,
		  //					  childrenToTest.at(j)->nodeIndex_,bins,buffer1);
		  //		  graphviz.write(buffer, sizeOfSprintf);
		  
		  //		}
		  
		  //		/*there are no left and right trees */
		  //		childrenToTest.at(j)->leftTree = 0;
		  //		childrenToTest.at(j)->rightTree = 0;
		  //		/* since no left and right children, the test must be set to NULL */
		  //		childrenToTest.at(j)->bestTest = 0;
		  //	  }
		}
		
		
		
		Test* temp_test = 0;
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
	  
	  //	  fprintf(stderr,"The total number of nodes grown is %d.\n", globalNodeIndex ); 
	  
	} 
	
	
	
	
	
    /**  
	 * It considers an integral Image (possibly the integral image of a test data image) and sends it down the tree and returns the bin which has maximum strength. If there are many bins with the same strength, the algorithm picks the first in order of increasing ids. ie. (0, 5, 5, 5, 0, 3) would return class 1. 
	 * It should be taken into account that the test integral images are assumed to spawn from images of the same size as the ones used for training 
	 * \param tree random tree 
	 * \param integralImage the integral image of the test sample 
	 * \param config ConfigClass Instance 
	 * \param faceProbabilityResult FMP value 
	 */
	int getBinOfClassification(Tree* tree, int* integralImage, ConfigClass &config, double &faceProbabilityResult) {
	  Test* test = 0;
	  bool childPass;
	  Tree *currentTree = tree, *prevTree = tree;
	  int histogramIndex = -1; //, maximumValue = -1, size = 0;
	  int nos_faces_path = 1;
	  int nos_nonfaces_path = 1; 
	  
	  
	  while(currentTree!=NULL){
		if(currentTree->leftTree==NULL || currentTree->rightTree==NULL){ 
		  if(currentTree->theIntegralImages == NULL) { 
			currentTree = prevTree; 
			break;
		  }
		  else
			break; 
		  
		  //	  if(currentTree->theIntegralImages == NULL)
		  //		break; 
		  //	  else
		  //		prevTree = currentTree;
		  //	  break;
		}
		
		test = currentTree->bestTest; 
		
		if(test == NULL){
		  currentTree = prevTree;
		  //	  prevTree = currentTree;
		  break;
		}
		
		childPass = test->imagePass(integralImage, test->xMain, test->yMain, test->xdash, test->ydash, test->theta1, test->theta2, config.imageInfoInstance.getHeight(), config.imageInfoInstance.getWidth(), test->haarFeature);
		prevTree = currentTree;
		
		if( childPass )
		{
		  nos_faces_path += test->leftBranch[0].size(); 
		  nos_nonfaces_path += test->leftBranch[1].size(); 
		}
		else
		{
		  nos_faces_path += test->rightBranch[0].size();
		  nos_nonfaces_path += test->rightBranch[1].size(); 
		}
		
		
		if(childPass)
		  currentTree = currentTree->leftTree;
		else
		  currentTree = currentTree->rightTree;
		
		
	  } 
	  int capacityOfBin = 0; 
	  //int numberOfFaces = 0;
	  
	  nos_faces_path += currentTree->theIntegralImages[0].iiVector.size(); 
	  nos_nonfaces_path += currentTree->theIntegralImages[1].iiVector.size(); 
	  
	  /*
	   for(int i=0;i<config.getNclasses();i++){
	   
	   size = currentTree->theIntegralImages[i].iiVector.size();
	   //	size = prevTree->theIntegralImages[i].iiVector.size();
	   capacityOfBin += size;
	   if(i==0) numberOfFaces = size;
	   //	if(maximumValue<size) { maximumValue = size; histogramIndex =  i;}
	   }
	   */ 
	  
	  /*
	   double faceProbability = (double)numberOfFaces/(double)capacityOfBin;
	   */
	  double faceProbability = (double)nos_faces_path/(double)(nos_nonfaces_path + nos_faces_path); 
	  
	  faceProbabilityResult = faceProbability; // This is stored in WindowClass instance
	  
	  if(WR_FACE_MAP_FILE) {
		if(capacityOfBin == 0)
		  config.faceMapFile<<" "<<-1<<" ";
		else
		  config.faceMapFile<<" "<<faceProbability<<" ";
          }
	  
	  test = 0;
	  currentTree = 0;
	  prevTree = 0;
	  
	  if(faceProbabilityResult > config.getFaceMapThreshold())	
		histogramIndex = 0; 
	  else
		histogramIndex = 1;
	  
	  return histogramIndex;
	  
	}
	
    /**  A detection module - identifies if the patch from the train image falls into a category ( a binary decision eg. face /non-face) as defined by category[] 
     * \param tree the random tree
     * \param integralImageWhole the integral image of the test image sample 
     * \param hOffset the iterative subwindow consdired: height offset 
     * \param wOffset the iterative subwindow considered: width offset 
     * \param config ConfigClass Instance 
     * \param trainHeight Height of the training images 
     * \param trainWidth Width of the training images 
     * \param testHeight height of the test images 
     * \param testWidth width of the test images 
     * \param category category of the face images 
     * \param indexOfFaceCategory the index in the category list 
     * \param faceProbabilityResult the FMP value 
     */ 
	bool detectIntegralImagePatch(Tree *tree, int* integralImageWhole, int hOffset, int wOffset, UserDef::ConfigClass& config, int trainHeight, int trainWidth, int testHeight, int testWidth, int* category, int indexOfFaceCategory, double &faceProbabilityResult){ 
	  
	  
	  if((hOffset+trainHeight)>testHeight) // Check boundaries
		return false;
	  if((wOffset+trainWidth)>testWidth)
		return false;
	  
	  int* integralImage = new int[trainHeight*trainWidth]; // Extract Patch
	  for(int h=0;h<trainHeight;h++)
		for(int w=0;w<trainWidth;w++)
		  integralImage[h*trainWidth + w] = integralImageWhole[(h+hOffset)*testWidth + (w+wOffset)];
	  
	  //2. Apply bin of classification to get category
	  // the exact subwindow is sent for clasification
	  int classifiedUnder=getBinOfClassification(tree, integralImage, config, faceProbabilityResult);
	  
	  //3. Check against category
	  bool answer = false; //the default is non-face
	  if(classifiedUnder == category[indexOfFaceCategory]) // It is a face 
		answer = true; 
	  
	  
	  delete[] integralImage; 
	  return answer; 
	  
	}
	
    /** The sliding window module. We assume we have a testIntegralImage and we have a integer array that highlights those classes that were faces and those that are not. We then create windows and send them over to detectIntegralImagePatch and see if there is a yes or a no. If there is a yes, an instance of WindowClass is created and pushed into the config.windows vector. This is repeated for all possible windows and we can retrieve the contents of config.windows to obtain all detected faces. 
	 *  This version of the sliding window protocol simply stores every possible detection
	 * \param tree the random tree
	 * \param testIntegralImage the integral image of the test sample 
	 * \param config the ConfigClass Instance 
	 * \param category the category of object classes 
	 * \param indexOfFaceCategory the index of the faces in the object classes 
	 * \param ForestIndex the index of the random tree in the random forest 
	 * \param scaleFactor the scale factor to scale the test images to 
	 */
	int slidingWindow(Tree *tree, int* testIntegralImage, UserDef::ConfigClass& config, int* category, int indexOfFaceCategory, int ForestIndex, double scaleFactor ){ 
	  
	  int trainHeight = config.imageInfoInstance.getHeight();
	  int trainWidth = config.imageInfoInstance.getWidth();
	  
	  const int testHeight = config.testImageInfoInstance.getHeight();
	  const int testWidth = config.testImageInfoInstance.getWidth();
	  
	  if(testHeight<trainHeight) 
	  { // the HEIGHT of the test rectangle is smaller than train rectangle
		
		//	printf("\nForest.hh@slidingWindow: testheight<trainheight: %d %d \n", testHeight, trainHeight);
		return -1;
	  }
	  
	  if(testWidth<trainWidth) { //the WIDTH of the test rectanlge is smaller than train rectangle
		
		//	printf("\nForest.hh@slidingWindow: testWidth<trainWidth: %d %d\n", testWidth, trainWidth);
		return -1;
	  }
	  
	  // Since there are numerous offsets (everything in the region), I am going to try to prevent this. 
	  int lastHOffset = -1;
	  int lastWOffset = -1;
	  
	  double faceProbabilityResult = -1.0;
	  
	  int subwindowRange = config.getSubwindowRange(); 
	  
	  if( ALLOW_WITSCALING ) { 
		subwindowRange = (int)( scaleFactor * subwindowRange ); 
		
		if( subwindowRange == 0 ) subwindowRange = 1; 
		
		if( subwindowRange > 7 ) subwindowRange = 7; 
	  }
	  
	  //	  int areaOfComparisonOfFaceProbability = config.getDetectionThreshold();
	  
	  for(int i = 0; i < (testHeight - 1); i += subwindowRange )	  
		for(int j = 0; j < (testWidth - 1); j += subwindowRange )  {
		  
		  // the subwindow is sent down the tree
		  if( detectIntegralImagePatch( tree, testIntegralImage, i, j, 
									   config, trainHeight, trainWidth, testHeight, 
									   testWidth, category, indexOfFaceCategory, 
									   faceProbabilityResult ) ) { 
			
			//		if((i<(lastHOffset + areaOfComparisonOfFaceProbability)) && (j<(lastWOffset +areaOfComparisonOfFaceProbability))) { // Range around detected face that will be studied
			
			//		  if(config.windows.size()>0){ 
			
			//		if((config.windows.back())->getFaceProbabilityResult()<faceProbabilityResult){ //BetterOne
			
			//			config.windows.pop_back();
			//			config.windows.push_back(new UserDef::WindowClass(i, j, trainHeight, trainWidth, faceProbabilityResult)); // New one pushed (dont change lastHOffset and lastWOffset values!)
			
			//		} 
			//		  } // if(size>0)
			//		} // if(i<lastHOffs...
			//		else 
			//		  { 
			config.windows.push_back( new UserDef::WindowClass( i, j, 
															   trainHeight, 
															   trainWidth, 
															   faceProbabilityResult ) );
			lastHOffset = i;
			lastWOffset = j;
			//		  }
		  }
		} // for(..
	  
	  Auxillary::writeToFile(config.windows, config.detectionsFile);
	  return 0;
	}
	
	
    /** a re-implementation of getBinOfClassification to deal with integral images of patches of larger size than the tree was trained for. 
     *That is, to work with slidingWindowWithScaling  
     * 
     * \param tree random tree
     * \param integralImage the integral image of the tree
     * \param config ConfigClass Instance
     * \param scaleFactor factor to scale test candidates to
     * \param faceProbabilityResult FMP value 
     */ 
	int scaledGetBinOfClassification(Tree* tree,int *integralImage, UserDef::ConfigClass& config, double scaleFactor, double &faceProbabilityResult)
	{ 
	  Test* test = 0;
	  bool childPass;
	  Tree *currentTree = tree, *prevTree = tree;
	  int histogramIndex = -1, size = 0;
	  
	  while(currentTree!=NULL){
		if(currentTree->leftTree==NULL || currentTree->rightTree==NULL){ 
		  if(currentTree->theIntegralImages == NULL) { 
			currentTree = prevTree; 
			break;
		  } 
		  else
			break;
		}
		
		test = currentTree->bestTest; 
		
		if(test == NULL){
		  prevTree = currentTree;
		  break;
		}
		
		// The theta1,2 do not need to be scaled since the normalization already takes care of this scaled patch using the height and breadth of the patch as parameters in the denominator of the normalization equation
		childPass = test->imagePass( integralImage, 
									(int)( scaleFactor * test->xMain),
									(int)( scaleFactor * test->yMain), 
									(int)(scaleFactor* test->xdash),
									(int)(scaleFactor*test->ydash), 
									test->theta1,
									test->theta2, 
									(int)(scaleFactor*config.imageInfoInstance.getHeight()),
									(int)(scaleFactor*config.imageInfoInstance.getWidth()), 
									test->haarFeature);
		prevTree = currentTree;
		if(childPass)
		  currentTree = currentTree->leftTree;
		else
		  currentTree = currentTree->rightTree;
	  } 
	  int capacityOfBin = 0; 
	  int numberOfFaces = 0;  
	  for(int i=0;i<config.getNclasses();i++){
		size = currentTree->theIntegralImages[i].iiVector.size();
		capacityOfBin += size;
		if(i==0) numberOfFaces = size;
		
	  }
	  
	  double faceProbability = (double)numberOfFaces/(double)capacityOfBin;
	  faceProbabilityResult = faceProbability; 
	  
	  if( WR_FACE_MAP_FILE ) {
		if(capacityOfBin == 0)
		  config.faceMapFile<<" "<<-1<<" ";
		else
		  config.faceMapFile<<" "<<faceProbability<<" ";
          }
	  
	  test = 0;
	  currentTree = 0;
	  prevTree = 0;
	  
	  
	  if(faceProbability>config.getFaceMapThreshold())
	  {	histogramIndex = 0; 
		//	  cout<<" "<<faceProbabilty<<" ";
	  }
	  else
		histogramIndex = 1;
	  
	  return histogramIndex;
	  
	}
	
    /** this works much like the slidingWindow module except that it incorporates consdering subwindows of arbitrary size and scaling them down to trainHeight and trainWidth to check down the tree. I implement the detectntegralImagePatch right here 
     * I am assuming the width == height for the face images ( need to change this if this is not true) 
     * \param tree the random tree
     * \param testIntegralImage the integral Image of the test sample
     * \param config ConfigClass Instance
     * \param category class index
     * \param indexOfFaceCategory specific granulairty of clas s index
     * \param scaleFactor factor to scale input image to
     */ 
	void slidingWindowWithScaling(Tree *tree, 
								  int *testIntegralImage, 
								  UserDef::ConfigClass& config, 
								  int *category, 
								  int indexOfFaceCategory, 
								  double scaleFactor) { 
	  
	  const int trainHeight = config.imageInfoInstance.getHeight();
	  const int trainWidth = config.imageInfoInstance.getWidth();
	  const int testHeight = config.testImageInfoInstance.getHeight();
	  const int testWidth = config.testImageInfoInstance.getWidth();
	  
	  
	  if(testHeight<trainHeight) { 
	    //		perror("~Forest.hh@slidingWindow: testheight<trainheight;");
	    LibLogger::log_error("Forest.hh","test_height < train_height "); 
		//		exit(1);
	  }
	  if(testWidth<trainWidth) {
	    LibLogger::log_error("Forest.hh","test_width < train_width"); 
	    //		perror("~Forest.hh@slidingWindow: testWidth<trainWidth");
	    //		exit(1);
	  }
	  
	  int lastHOffset = -1;
	  int lastWOffset = -1;
	  double faceProbabilityResult = -1.0;
	  //	  int areaOfComparisonOfFaceProbability = config.getDetectionThreshold();
	  
	  int trainH = (int)(scaleFactor * trainHeight); // for the integral images of a "scaled" patch
	  int trainW = (int)(scaleFactor * trainWidth);
	  
	  for(int i = 0; i < (testHeight - 1); i += config.getSubwindowRange() )	  
		for(int j = 0; j < (testWidth - 1); j += config.getSubwindowRange() )  {
		  
		  
		  if( (trainH + i) > testHeight )
			break;
		  
		  if( (trainW + j) > testWidth )
			break; 
		  
		  int *integralImage = new int[trainH*trainW];
		  
		  for(int h=0;h<trainH;h++)
			for(int w=0;w<trainW;w++)
			  integralImage[h*trainW + w] = testIntegralImage[(h+i)*testWidth + (w+j)];
		  
		  
		  int classifiedUnder = scaledGetBinOfClassification(tree, integralImage, config, scaleFactor, faceProbabilityResult);
		  
		  bool answer = false;
		  if(classifiedUnder == category[indexOfFaceCategory]) answer = true;
		  
		  if(answer) { 
			
			config.windows.push_back( new UserDef::WindowClass( i, j, 
															   trainH, 
															   trainW, 
															   faceProbabilityResult ) );
			lastHOffset = i;
			lastWOffset = j;
			//		  }
		  }
		} // for(..
	  
	  
	  Auxillary::writeToFile(config.windows, config.detectionsFile);
	}
	
	
	
	
	
    /** a test unit for: detectIntegralImagePatch(Tree *tree, int* integralImageWhole, int hOffset, int wOffset, UserDef::ConfigClass& config, int trainHeight, int trainWidth, int testHeight, int testWidth, int* category, int categorySize)
	// The category and category size are dummy 
	// Also emulates RandomTree.cpp:runClassification
	//STATUS: detectIntegralImagePatch performs as expected 
	* \param tree random tree
	* \param config ConfigClass Instance
	*/ 
	void TestDetectIntegralImagePatch(Tree* tree, UserDef::ConfigClass& config){
	  int correctClassified = 0;
	  int totalNumberOfImages = 0;
	  int* category = new int[1];
	  bool classifyResult;
	  
	  for(int classIndex = 0;classIndex<config.getNclasses();classIndex++){ 
		category[0] = classIndex;
		for(unsigned int imageIndex = 0;imageIndex<config.testIntegralImages[classIndex].iiVector.size();imageIndex++){ 
		  totalNumberOfImages++;
		  double faceProbabilityResult;
		  classifyResult = 
		  detectIntegralImagePatch(tree, 
								   (config.testIntegralImages[classIndex].iiVector.at(imageIndex)), 
								   0, 0, config, config.imageInfoInstance.getHeight(), 
								   config.imageInfoInstance.getWidth(), 
								   config.imageInfoInstance.getHeight(), 
								   config.imageInfoInstance.getWidth(), category, 1, 
								   faceProbabilityResult);
		  if(classifyResult)
			correctClassified++;
		}
	  }
	  
	  cout<<"Correctly classified are"<<correctClassified<<" out of "<<totalNumberOfImages;
	  
	
	}
	
	
  };
  
	/*
  int writeToFile( Forest::Test *test, std::FILE *fp ) { 
	
	fprintf( fp, "%d \t %d \t %d \t %d ", test->xMain, test->yMain, 
			test->xdash, test->ydash ); 
	
	fprintf( fp, "%f \t %f ", test->theta1, test->theta2 ); 
	
	
	
	
	for( int i = 0; i < NCLASSES; i++ ) { 
	  
	  fprintf( fp, "%d\n", test->leftBranch[i].size() ); 
	  for( int j = 0; j < test->leftBranch[i].size(); j++ )
		fwrite( test->leftBranch[i].iiVector.at( j ) , 1,  sizeof( int* ), fp ); 
	  
	  fprintf( fp, "%d\n", test->rightBranch[i].size() ); 
	  for( int j = 0; j < test->rightBranch[i].size(); j++ )
		fwrite( test->rightBranch[i].iiVector.at( j ) , 1, sizeof( int* ), fp ); 
	  
	}
	
  }
  
*/
	/*
  int readFromFile( Forest::Test *test, std::FILE *fp ) { 
	
	int temp;
	
	fscanf( fp, "%d", &temp ); test->xMain = temp; 
	fscanf( fp, "%d", &temp ); test->yMain = temp;
	fscanf( fp, "%d", &temp ); test->xdash = temp;
	fscanf( fp, "%d", &temp ); test->ydash = temp;
	
	float tempf;
	
	fscanf( fp, "%f", &tempf ); test->theta1 = tempf; 
	fscanf( fp, "%f", &tempf ); test->theta2 = tempf; 
	
	
	int size;
	int *ptr; 
	
	for( int i = 0; i < NCLASSES ; i++ ) { 
	  
	  fscanf( fp, "%d", &size ); 
	  for( int j = 0; j < size; j++ ) { 
		
		fread( ptr, 1, sizeof( int* ), fp); 
		test->leftBranch[i].iiVector.push_back( ptr ); 
	  }
	  
	  
	  fscanf( fp, "%d", &size ); 
	  for( int j = 0 ; j  < size ; j++ ) { 
		
		fread( ptr, 1, sizeof( int* ), fp ); 
		test->rightBranch[i].iiVector.push_back( ptr );
		
	  }
	  
	  
	}
	
	return 0;
  }
  
  int writeToFile( Forest::Tree* root, std::FILE *fp ){ 
	
	using Forest::Tree; 
	size_t treeptr = sizeof( Tree* ); 
	
	if( root != 0 ) { 
	  
	  fprintf( fp, "1\n" ); // denote node not null, while reading
	  fwrite( root, treeptr, 1, fp ); 
	} 
	
	else { 
	  fprintf( fp, "-1\n" );
	  return 0;
	}
	
	fprintf( fp, " %d\n", root->nodeIndex_ ); 
	fprintf( fp, "%d\n", root->parentNodeIndex_ );
	fprintf( fp, "%d\n", root->level_ ); 
	
	if( root->bestTest != 0) { 
	  
	  fprintf( fp, "1\n" );
	  writeToFile( root->bestTest, fp ); 
	}
	else { 
	  
	  fprintf( fp, "-1\n" ); 
	  return 0;
	  
	}
	
	
	if( root->leftTree !=0 ) writeToFile( root->leftTree , fp ); 
	else fprintf( fp, "-1\n" ); 
	
	if( root->rightTree !=0 ) writeToFile( root->rightTree , fp ); 
	else fprintf( fp, "-1\n" ); 
	
	return 0; 
	
  }
  
*/



  /** propagate the integral images down the tree 
   * \param tree the random tree
   * \param config ConfigClass Instance
   * \param identityII the integral image
   * \param height of the original image
   * \param width of the original image
   */  
  int propagateTree( Tree *tree, ConfigClass &config, int* identityII, int height, int width )
  {
	Tree *root = tree; 
	Tree* child2Propagate = 0; 
	Test *root_test = root->bestTest; 
	bool identityFound = false; 
	
	using UserDef::compareIntegralImages;
	
	if( root->leftTree == 0 && root->rightTree != 0 )
	{
	  return propagateTree( root->rightTree, config, identityII, height, width);   
	}
	
	if( root->leftTree !=0 && root->rightTree == 0 )
	{
	  return propagateTree( root->leftTree, config, identityII, height, width ); 
	}
	
	if( root_test == 0 ) 
	{
	  printf("the reached the leaf node(stop) and the node index is %d\n", root->getUniqueNumber() );
	  return 0; 
	}
	
	//check in the left bin (only for faces - obviously no need to consider class#1)
	for( int i = 0; i < root_test->leftBranch[0].size(); i++ )
	{
	  int* leftII = root_test->leftBranch[0].iiVector.at(i); 
	  
	  bool comparison = false; 
	  comparison = compareIntegralImages( leftII, identityII, height, width ); 
	  
	  if( comparison )
	  {
		identityFound = true; 
		break; 
	  }
	  
	  
	}
	
	if( identityFound )
	  child2Propagate = root->leftTree;
	else
	  child2Propagate = root->rightTree;
	
	if( child2Propagate == 0 ) 
	{
	  printf("I have reached nodeIndex(above) %d\n", root->getUniqueNumber() );
	  return 0;
	}
	
	return propagateTree( child2Propagate, config, identityII, height, width ); 
	
  }
  
  
  /** get classification results for the integral image on the random tree 
   * \param aTree the random tree
   * \param config ConfigClass Instance 
   * \param ii integral image
   */ 
  int runClassification( Tree* aTree, ConfigClass& config, int* ii )
  {
	
	Tree *root = aTree, *prevTree = aTree; 
	bool childPass; 
	Test* root_test = 0;
	
	//int histogramIndex = -1;
	
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
	  
	  childPass = root_test->imagePass( ii, root_test->xMain, root_test->yMain, root_test->xdash, root_test->ydash, root_test->theta1, root_test->theta2, config.imageInfoInstance.getHeight(), config.imageInfoInstance.getWidth(), root_test->haarFeature ); 
	  
	  prevTree = root; 
	  
	  if( childPass ) 
		root = root->leftTree; 
	  else
		root = root->rightTree; 
	  
	}
	
	
	//since we are dealing with a multiclassproblem, we find the maximum number of bins

	int max = -1; 
	int index = -2; 
	int total_size = 0; 
	
	for( int i = 0; i < config.getNclasses(); i++ ) 
	{
	  //	  printf("the sizes are %zu\n", root->theIntegralImages[i].iiVector.size() ); 
	  
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
	
 
	/* working*/
	/* int max = root->theIntegralImages[0].iiVector.size(); 
	 int total_size = max + root->theIntegralImages[1].iiVector.size(); 
	 
	printf("\n %d %d \n", root->theIntegralImages[0].iiVector.size(), root->theIntegralImages[1].iiVector.size() ); 
	
	 double probabilityOfHighestClass = (double)max/(double)total_size; 
	 
	 if( probabilityOfHighestClass > 0.5 ) 
	 return 0; 
	 else
	 return -1;
	*/
	
  }
  

  /** get classification results for any random tree 
   * \param aTree the random tree
   * \param ii the integral image
   * \param train_height height of the training images
   * \param train_width widht of the trainign images
   * \param nclasses number of indentities 
   */ 
  int runClassificationForArbitraryTree( Tree* aTree, int* &ii, int train_height, int train_width, int nclasses )
  {
	
	Tree *root = aTree, *prevTree = aTree; 
	bool childPass; 
	Test* root_test = 0;
	
	//	int histogramIndex = -1;
	
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
	
	
	//since we are dealing with a multiclassproblem, we find the maximum number of bins
	
	int max = -1; 
	int index = -2; 
	int total_size = 0; 
	
	for( int i = 0; i < nclasses; i++ ) 
	{
	  //	  printf("the sizes are %zu\n", root->theIntegralImages[i].iiVector.size() ); 
	  
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
	
	
	/* working*/
	/* int max = root->theIntegralImages[0].iiVector.size(); 
	 int total_size = max + root->theIntegralImages[1].iiVector.size(); 
	 
	 printf("\n %d %d \n", root->theIntegralImages[0].iiVector.size(), root->theIntegralImages[1].iiVector.size() ); 
	 
	 double probabilityOfHighestClass = (double)max/(double)total_size; 
	 
	 if( probabilityOfHighestClass > 0.5 ) 
	 return 0; 
	 else
	 return -1;
	 */
	
  }
  
  
  /** return the classsification results for an IplImage -i.e. integral image calculated here 
   * \param aTree the random tree
   * \param imageToClassify the image to classify/recognize
   * \param train_height height of the training images
   * \param train_width width of the training images
   * \param nclasses number of identities 
   */ 
  int returnClassLabelForIplImage( Forest::Tree *aTree, IplImage *imageToClassify, int train_height, int train_width, int nclasses )
  {
	
	int* ii = new int[ imageToClassify->height * imageToClassify->width ]; 
	UserDef::calculateIntegralImage( imageToClassify, ii );
	
	
	return runClassificationForArbitraryTree( aTree, ii, train_height, train_width, nclasses  ) ; 
	
  }
  
  using UserDef::ConfigClass;
  
  /** grow a random tree quickly, given the location of the training samples 
   * \param train_images images from the training samples
   * \param nclass number of identities 
   * \param root root node of the random tree
   * \param train_height height of the training samples
   * \param train_width width of the training images
   * \param config ConfigClass Instance 
*/ 
  void growTreeFromImages( char* train_images, int nclasses, Forest::Tree* &root, int &train_height, int &train_width, ConfigClass& config  )
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
			
			UserDef::calculateIntegralImage( tempImage, tempII );
			
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
	root = new Forest::Tree( config, globalNodeIndex, numberOfTreesGrown++ ); 

		
  }
  
  
  /** build a forest quickly, given the location of the training smaples 
   * \param train_images training samples
   * \param nclass number of identities 
   * \param _forest the random forest 
   * \param train_height height of the training images
   * \param train_width width of the training images
   * \param config Config Class Instance 
   */ 
  void makeForest( char* train_images, int nclasses, vector<Tree*>& _forest, int &train_height, int &train_width, ConfigClass& config )
  {
  
//  UserDef::ConfigClass config( nclasses );

  for(int i = 0; i < nclasses; i++ ) 
  { 
    Forest::Tree* tempTree; 
    int t_height, t_width; 
    int _globalNodeIndex = 0;
    int _numberOfTrees = 0;
    if( i == 0 )
        growTreeFromImages( train_images, nclasses, tempTree, t_height, t_width, config ); 
    else 
    {
        _globalNodeIndex = 0 ;
        _numberOfTrees = i++; 
        tempTree = new Tree( config, _globalNodeIndex, _numberOfTrees ); 
    }
    
    _forest.push_back( tempTree ); 
    
   }
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
     * \param train_images the training samples 
     * \param nclasses number of identities 
     * \param train_height height of the training samples 
     * \param train_width widht of the trainig images 
     * \param config ConfigClass Instance 
     * \param forestSize size of the forest 
     */ 
    ForestClass( char* train_images, int nclasses, int &train_height, int &train_width, ConfigClass& config, int forestSize ) 
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
   
     // printf("The number of trees grown are %d", _forest.size() ); 
   
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
     * \param i get the tree of the index in the forest 
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

//      for( int i=0; i < _forestSize; i++ ) 
//     { 
//         Tree* tempTree = _forest.at(i); 
//         delete tempTree; 
        
//     }
    
//     _forest.clear(); 
   
  } 
  };
  
  
  /** for a given iplimage, return majority voting on random forest 
   * \param forestClassInstance an instance of the forest class 
   * \param inputImage the test sample 
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
     
  
  
  
  
  //	 int readFromFile( Forest::Tree* tree, std::File *fp ) { 
  
  //	   using Forest::Tree; 
  
  //	   size_t treeptr = sizeof( Tree* ); 
  
  //	   int status;
  
  //	   fscanf( fp, "%d", &status ); 
  
  //	   if( status == -1 ) { 
  //		 tree = 0;
  //		 return 0;
  //	   }
  
  //	   fread( tree, 1, treeptr, fp ); 
  
  //	   fscanf( fp, "%d", &status ); root->nodeIndex_ = status; 
  //	   fscanf( fp, "%d", &status ); root->parentNodeIndex_ = status;
  //	   fscanf( fp, "%d", &status ); root->level_ = status; 
  
  //	   fscanf( fp, "%d", &status); 
  
  //	   if( status == -1 ) { 
  
  //		 root->bestTest = 0; 
  //		 root->leftTree = 0 ;
  //		 root->rightTree = 0; 
  //		 return 0; 
  //	   }
  
  //	   readFromFile( root->bestTest, fp ); 
  
  
  //	   fscanf( fp, "%d", &status ); 
  
  //	   if( status == -1 ) { 
  
  //		 root->leftTree = 0; 
  
  //	   }
  //	   else
  //		 readFromFile( root->leftTree, fp ); 
  
  
  //	   fscanf( fp, "%d", &status ); 
  
  //	   if( status == -1 ) { 
  
  //		 root->rightTree = 0; 
  //	   }
  //	   else	   
  //		 readFromFile( root->rightTree, fp ); 
  
  //	   return 0; 
  
  //	 }
}
#endif
