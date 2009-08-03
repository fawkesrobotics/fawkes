
/***************************************************************************
 *  qa_randomtree.h - QA for random tree
 *
 *  Generated: Mon Apr 21 11:49:34 2008
 *  Copyright  2008  Vaishak Belle
 *
 *****************************************************************************/

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

/// @cond QA
#include <string> 
#include <opencv/cv.h>
#include <opencv/highgui.h> 
#include <fvutils/recognition/forest/forest.h>
#include <fvutils/recognition/forest/forest_aux.h>
#include <fvutils/recognition/forest/forest_param_default.h>

//the main functions





// class MergeWindowClass : public UserDef::WindowClass { 
  
// private: 
//   bool considered ; 
  
// public:
//   void setAsConsidered() { considered = true; } 
  
//   bool isConsidered() const { return considered; } 
  
//   MergeWindowClass( int h, int w, int H, int W, double p ) : WindowClass( h, w, H, W, p), 
//   considered(false) { }
// }; 


// typedef struct ObjectPos
// {
//   float x;
//   float y;
//   float width;
//   int found;    /* for reference */
//   int neighbors;
// } ObjectPos;



//int Forest::Tree::TreeLevel = 0;

// using Forest::Tree; 
// using Forest::Test; 
// using Forest::propagateTree;


// /* Unit Testing module */


// /* _working_ */
// void _tTest(UserDef::ConfigClass &config) { 
  
//   using Forest::Test; 
  
//   Test *sampleTest;
//   sampleTest = new Test( 8, 9, config ); 
  
//   sampleTest->runTestsOnIntegralImages( config.integralImages, 
// 					0, 
// 					config );
  
  
//   delete sampleTest; 
  
// }


/* _working_ */
// void _tTree( UserDef::ConfigClass &config ) { 
  
//   using Forest::Tree; 
//   int globalNodeIndex = 0; 
  
//   Tree* aTree = new Tree( config, 
// 						 globalNodeIndex,
// 						 1 ); 
  
  
//   delete aTree; 
  
  
// }


// /* _not_checked_ */
// void dealloc_mem( Forest::Tree* root ) { 
  
  
//   ERR_MODULE("dealloc_mem"); 
//   exit(1); 
  
//   /*
//    if( root == 0 ) 
//    return ;
   
//    Test* test = root->bestTest; 
   
//    if( test == 0 ) 
//    return; 
   
//    for( int i = 0 ; i < NCLASSES ; i++ ) 
//    test->leftBranch[i].iiVector.clear(); 
   
//    for( int i = 0 ; i < NCLASSES ; i++ ) 
//    test->rightBranch[i].iiVector.clear(); 
   
//    Tree* leftTree = root->leftTree;
//    Tree* rightTree = root->rightTree; 
   
//    if( leftTree != 0 ) 
//    dealloc_mem( leftTree ); 
   
//    if( rightTree != 0 )
//    dealloc_mem( rightTree ); 
//    */
  
// } 


// /* _working_ */
// int writeToFile(std::ostream& out, Forest::Test& test){ 
//   out<<test.theta1<<endl;
//   out<<test.theta2<<endl;
//   out<<test.xdash<<endl;
//   out<<test.ydash<<endl;
//   out<<test.xMain<<endl;
//   out<<test.yMain<<endl;
//   out<<test.haarFeature<<endl;
//   return 0; 
// }

// /* _working_ */
// int readFromFile(std::ifstream& in, Forest::Test& test){ 
//   in>>test.theta1;
//   in>>test.theta2;
//   in>>test.xdash;
//   in>>test.ydash;
//   in>>test.xMain;
//   in>>test.yMain;
//   in>>test.haarFeature;
//   return 0; 
// }




/*****************************************
 
 writeToFile
 
 ****************************************/ 

// int writeToFile( std::ostream& out, std::ostream& testOut, Forest::Tree& tree ){ 

//   /* The write to file is as per the following simple rules:
//    * 1. If it is a null node, i write -1
//    * 2. If the node is a leaf node, then I write to the traintree.dat file, -2 times nodeIndex - 
//    * this is because when I read back index, if they happen to below 0 && !=-1, I can distinguish leaf nodes
//    * 3. If the node is not a null node and not a leaf node, i write down the nodeIndex as is.  
//    */

//   if(tree.bestTest == 0){ 
//     // If this is indeed a leaf node, we need theIntegralImages[0,1].iiVector.size()'s. So, what we do is write -2*nodeIndex to the file. While reading, if it encounters <0 && !=-1, then we ask it to read the sizes instead of the normal test instance. See the corresponding readFromFile for more implementation details.  

//     out<<-2*tree.nodeIndex_<<endl; // this is a leaf node - no best test 
//     cout<<tree.theIntegralImages[0].iiVector.size()<<" "<<tree.theIntegralImages[1].iiVector.size()<<" ";

//     out<<tree.theIntegralImages[0].iiVector.size()<<endl;
//     out<<tree.theIntegralImages[1].iiVector.size()<<endl;
//   }

//   else {  

//     out<<tree.nodeIndex_<<endl;
//     writeToFile(testOut, *(tree.bestTest));
//   }

//   if(tree.leftTree!=0) 
//     writeToFile(out, testOut, *(tree.leftTree));

//   else 
//     out<<-1<<endl;

//   if(tree.rightTree!=0) 
//     writeToFile(out, testOut, *(tree.rightTree));

//   else 
//     out<<-1<<endl;
// } 



/*****************************************
 
 readFromFile
 
 ****************************************/ 

// int readFromFile(std::ifstream& in, std::ifstream& testIn, Forest::Tree** tree){

//   int nodeIndex;

//   // Read node index
//   in>>nodeIndex;

//   // Check if it supposed to be a null node (not a leaf node but a null node)
//   if(nodeIndex == -1) 
//     *tree = 0;

//   if(nodeIndex<0 && nodeIndex!=-1){ 

//     nodeIndex /=-2;

//     UserDef::VectorOfIntegralImages* integralImages = new UserDef::VectorOfIntegralImages[2];

//     integralImages[0].iiVector.clear();
//     integralImages[1].iiVector.clear();

//     if(*tree==0) { 

//       *tree = new Forest::Tree(integralImages,0,0,nodeIndex,0);
//       // (*tree)->leftTree = (*tree)->rightTree = 0;
//       // 	(*tree)->bestTest = 0;
//     }

//     else { 

//       //(*tree)->theIntegralImages = new UserDef::VectorOfIntegralImages[NCLASSES];

//       for(int i=0;i<NCLASSES;i++)
// 	(*tree)->theIntegralImages[i].iiVector.assign( 
// 						      integralImages[i].iiVector.begin(), 
// 						      integralImages[i].iiVector.end());
//     }

//     (*tree)->nodeIndex_ = nodeIndex;

//     int size1=0, size2=0;

//     in>>size1;
//     in>>size2;

//     (*tree)->theIntegralImages[0].iiVector.resize(size1,0);
//     (*tree)->theIntegralImages[1].iiVector.resize(size2,0);

//     readFromFile(in, testIn, &((*tree)->leftTree)); // this will most definitely be a nullnode
//     readFromFile(in, testIn, &((*tree)->rightTree));// So Will this
//   }

//   else{ 

//     if(*tree == 0){ 

//       // 	UserDef::VectorOfIntegralImages* integralImages = new UserDef::VectorOfIntegralImages[2];
//       // 	integralImages[0].iiVector.clear();
//       // 	integralImages[1].iiVector.clear();

//       *tree = new Forest::Tree(0,0,0,nodeIndex,0);

//       // 	(*tree)->leftTree = (*tree)->rightTree = 0;
//       // 	(*tree)->theIntegralImages = 0;
//       // 	(*tree)->bestTest = 0;
//     }

//     else
//       (*tree)->nodeIndex_ = nodeIndex;

//     // Alloc mem for test
//     (*tree)->bestTest = new Forest::Test();

//     //read test instance
//     readFromFile(testIn,*((*tree)->bestTest));

//     //Repeat procedure for left and right tree
//     readFromFile(in, testIn, &((*tree)->leftTree));
//     readFromFile(in, testIn, &((*tree)->rightTree));

//   }
// }




/*****************************************
 
 initImages
 
 ****************************************/ 

// void initImages( UserDef::ConfigClass& config, char* trainImages, bool trainLocGiven ){



//   string* s = new string( trainImages ); 

//   if( trainLocGiven) 
//     config.setTrainImages( *s ); 

//   else  
//     config.setTrainImages();

//   IplImage* tempImage = 0; 
//   DIR* dir = 0;

//   // Holds file name for reading images. 
//   char itoaString[20];

//   int i = 0, j = 0;

//   // Set to false if loaded file is not an image.
//   bool flagNotImage = false;

//   struct dirent *ent = 0;
//   string mainDirPath = config.getTrainImages();  
//   string fileName , dirPath;


//   /** 
//    * The object images will be populated with images from each 
//    * directory specified 
//    * for a class across all classes. ie. 
//    * images[6] is the vector of IplImages 
//    * specified for the class 6. 
//    */
//   for(i = 0;i<NCLASSES;i++){
//     j = 0;


//     sprintf(itoaString,"%d",i);
//     dirPath = mainDirPath + itoaString;

//     if((dir = opendir(dirPath.c_str())) == NULL){

//       perror("Directory does not exist");
//       exit(0);
//     }

//     bool setBorders = false; 

//     while((ent = readdir(dir))!=NULL){

//       fileName = dirPath + "/" + ent->d_name;

//       tempImage = cvLoadImage(fileName.c_str());

//       if(!tempImage){

// 	flagNotImage = true;
//       }

//       if( !flagNotImage ) { 

// 	j++;

// 	//	  config.images[i].push_back(tempImage);

// 	config.imageInfoInstance.setHeight( tempImage->height );
// 	config.imageInfoInstance.setWidth( tempImage->width );

// 	int *tempII = new int[ ( tempImage->height ) * ( tempImage->width ) ];
// 	UserDef::calculateIntegralImage(tempImage, tempII);
// 	config.integralImages[i].iiVector.push_back( tempII );


// 	if( !setBorders ) { 
// 	  config.integralImages[i].setHeight( tempImage->height ); 
// 	  config.integralImages[i].setWidth( tempImage->width ); 
// 	  setBorders = true; 
// 	}

// 	//#OK#	cout<<j<<" "<<fileName.c_str()<<endl; 

// 	cvReleaseImage( &tempImage ); 
//       } // if( !flag..  

//       flagNotImage = false;
//     } // while(.. 
//   } // for( i.. 

//   //Write to file
//   config.setTrainIntegralImagesLoc(TRAINDT);

//   // Replicate the integral Images
// //   UserDef::VectorOfIntegralImages vectorOfIntegralImages[ NCLASSES ];
// //   for( int i = 0 ; i < NCLASSES ; i++ ) 
// //     for( int j = 0; j < config.integralImages[i].size() ; j++ ) 
// //       vectorOfIntegralImages[i].iiVector.push_back( &( config.integralImages[i].iiVector.at(j) ) ); 

//   Auxillary::writeToFile(config.integralImages , 
// 			 config.getTrainIntegralImagesLoc(), 
// 			 config.imageInfoInstance.getHeight(), 
// 			 config.imageInfoInstance.getWidth());

//   dir = 0;
//   ent = 0;


// } 


/*****************************************
 
 initTestImages
 
 ****************************************/
// void initTestImages(UserDef::ConfigClass &config){

//   config.setTestImages();

//   IplImage* tempImage = 0;
//   DIR* dir = 0;
//   char itoaString[20];

//   int i =0, j=0;

//   bool flagNotImage = false;

//   struct dirent *ent = 0;

//   string mainDirPath = config.getTestImages();

//   string fileName, dirPath;
//   int *tempII = 0;

//   for(i = 0;i<NCLASSES;i++){

//     j = 0;
//     sprintf(itoaString,"%d",i);
//     dirPath = mainDirPath + itoaString;

//     if((dir = opendir(dirPath.c_str())) == NULL){

//       perror("Directory does not exist");
//       exit(0);
//     } 


//     while((ent = readdir(dir))!=NULL){
//       fileName = dirPath + "/" + ent->d_name;

//       tempImage = cvLoadImage(fileName.c_str());

//       if(!tempImage){

// 	flagNotImage = true;
//       }

//       if(!flagNotImage){ 

// 	j++;
// 	//	  config.testImages[i].push_back(tempImage);
// 	int height = tempImage->height;
// 	int width = tempImage->width;

// 	tempII = new int[height*width];
// 	calculateIntegralImage(tempImage,tempII);

// 	config.testIntegralImages[i].iiVector.push_back( tempII );

// 	cvReleaseImage( &tempImage ); 
//       }

//       flagNotImage = false;
//     }
//   }

//   // Write down contents in a file
//   config.setTestIntegralImagesLoc(TESTDT);

// //   UserDef::VectorOfIntegralImages vectorOfIntegralImages[ NCLASSES ];

// //   for( int i = 0 ; i < NCLASSES ; i++ )
// //     for( int j = 0 ; j < config.testIntegralImages[i].size() ; j++ )
// //       vectorOfIntegralImages[i].iiVector.push_back( &( config.testIntegralImages[i].iiVector.at(j) ) ); 

//   Auxillary::writeToFile( config.testIntegralImages , 
// 			 config.getTestIntegralImageDataLoc(), 
// 			 config.imageInfoInstance.getHeight(), 
// 			 config.imageInfoInstance.getWidth());

//   tempImage = 0;
//   dir = 0;
//   ent = 0;
//   tempII = 0;

// }



/*****************************************
 
 
 runclassification
 
 
 ****************************************/
// void runClassification(Forest::Tree* tree, UserDef::ConfigClass& config){

//   int correctClassified = 0;
//   int totalNumberOfImages = 0;
//   int classifyResult = -1;

//   for(int classIndex = 0;classIndex<NCLASSES; classIndex++)

//     for(int imageIndex = 0;imageIndex<config.testIntegralImages[classIndex].iiVector.size();imageIndex++){

//       totalNumberOfImages++;
//       double faceProbabilityResult = -1.0;

//       classifyResult = tree->getBinOfClassification(tree, 
// 						    (config.testIntegralImages[classIndex].
// 						      iiVector.at(imageIndex)), 
// 						    config, faceProbabilityResult);

//       if(classifyResult == classIndex)
// 	correctClassified++;
//     }

//   cout<<"Correctly classified are"<<correctClassified<<" out of "<<totalNumberOfImages;
//   cout<<"The error rate is "<<(1.0 - (double)correctClassified/(totalNumberOfImages*1.0))<<endl;

// } 



// // Common module for handling filterNoise1234. 
// void filterNoise( vector< UserDef::WindowClass* > &forestWindows, UserDef::ConfigClass &config ) { 
  
  
//   using UserDef::filterNoise1; 
//   using UserDef::filterNoise2; 
//   using UserDef::filterNoise3;
//   using UserDef::filterNoise4;
//   using UserDef::filterNoise5; 
  
//   //filterNoise3( forestWindows, config, 1.0 );
//   filterNoise3( forestWindows, config, 1.0 ); 
//   filterNoise3( forestWindows, config, 1.0 ); 
//   //  filterNoise5( forestWindows, config, 1.0, 1 ); 
//   //  filterNoise5( forestWindows, config, 1.0 ); 
//   // filterNoise5( forestWindows, config, 1.0 ); 
//   // filterNoise5( forestWindows, config, 1.0 );
  
  
// }


// void final(UserDef::ConfigClass& config){
  
//   //   IplImage* tempImage = 0;
//   for(int i=0;i<config.getNclasses();i++){ 
	
//     //     while(config.images[i].size()!=0){./Ra
	
//     //       tempImage = config.images[i].back();
//     //       cvReleaseImage(&tempImage);
//     //       config.images[i].pop_back();
//     //     }
    
// 	//     for( int j = 0 ; j < config.integralImages[i].size() ; j++ )
// 	//       if( config.integralImages[i].iiVector.at(j) != 0 )
// 	// 	delete[] config.integralImages[i].iiVector.at( j ); 
    
	
//     config.integralImages[i].iiVector.clear();
//   }
  
//   //  delete[] config.integralImages;
  
//   for(int i=0;i<config.getNclasses();i++){
	
//     //     while(config.testImages[i].size()!=0){
//     //       tempImage = config.testImages[i].back();
//     //       cvReleaseImage(&tempImage);
//     //       config.testImages[i].pop_back();
//     //     }
    
// 	//     for( int j = 0 ; j < config.testIntegralImages[i].size() ; j++ ) 
// 	//       if( config.testIntegralImages[i].iiVector.at(j) != 0 )
// 	// 	delete[] config.testIntegralImages[i].iiVector.at(j); 
	
//     config.testIntegralImages[i].iiVector.clear();
    
//   }
  
  
//   UserDef::WindowClass* wc = 0; 
//   for( unsigned int i = 0 ; i < config.forestWindows.size() ; i++ ) { 
//     wc = config.forestWindows.at( i ) ;
//     delete wc; 
//   }
  
//   config.forestWindows.clear(); 
  
//   for( unsigned int i = 0 ; i < config.windows.size() ; i++ ) { 
//     wc = config.windows.at( i ) ; 
//     delete wc; 
//   } 
  
//   config.windows.clear(); 
  
  
//   //  delete[] config.testIntegralImages;
  
//   //   tempImage = 0;
// } 



// /* _working_ ? */
// /* heuristic to combine votes for sliding window with scaled tests */
// void combineVotes_2( UserDef::ConfigClass &config, int range) {   // this is for vote==2
  
  
//   /* have to finish up: 
//    * 1. fix up "range" variable to be rightly scaled 
//    * so the modified fucntion is
//    * com* ( config, range, scaleFactor )
//    * 2. correct vote==2 proc
//    */
  
//   ERR_MODULE( "combinevotes_2" ); 
//   exit( 1 );
  
  
//   //   if(config.forestWindows.size()==0) // If the size is 0, put everything in 
  
//   //     for(int i=0;i<config.windows.size();i++) 
  
//   //       config.forestWindows.push_back( new UserDef::WindowClass(
//   // 							       config.windows.at(i)->getY(),
//   // 							       config.windows.at(i)->getX(),
//   // 							       config.windows.at(i)->getHeight(),
//   // 							       config.windows.at(i)->getWidth(),
//   // 							       config.windows.at(i)->
//   // 							       getFaceProbabilityResult() ) ); 
  
//   //   else {
  
//   //     // In case the size is not 0, then we run through the set of detections we have in forestWindows. 
//   //     // If we find something in the neighborhood, then we choose the best Window between 
//   //     // config.windows' one and config.forestWindows' one. 
//   //     // If however, for an entry of config.windows, not a single forestWindows is member, then put 
//   //     // the config.windows' entry to forestWindows
  
//   //     using Auxillary::inRange;
  
//   //     using UserDef::WindowClass;
  
  
  
//   //     vector< MergeWindowClass* > original; 
//   //     vector< WindowClass* > temp; 
  
//   //     for( int i = 0; i < config.forestWindows.size(); i++ ) 
  
//   //       original.push_back( new MergeWindowClass( config.forestWindows.at(i)->getY(), 
//   // 						config.forestWindows.at(i)->getX(),
//   // 						config.forestWindows.at(i)->getHeight(),
//   // 						config.forestWindows.at(i)->getWidth(),
//   // 						config.forestWindows.at(i)->
//   // 						getFaceProbabilityResult() ) ); 
  
  
  
//   //     for(int i=0;i<config.windows.size();i++) { 
  
//   //       bool thereIsNeighbor = false; 
  
//   //       for(int j=0;j<original.size();j++) { 
  
//   // 	if(	       
//   // 	   inRange( config.windows.at(i)->getX(), 
//   // 		    original.at(j)->getX(), 
//   // 		    range )  && 
//   // 	   inRange( config.windows.at(i)->getY(), 
//   // 		    original.at(j)->getY(), 
//   // 		    range ) ) { 
  
//   // 	  thereIsNeighbor = true;  // Now, we choose the best neighbor 
  
//   // 	  if( config.windows.at(i)->getFaceProbabilityResult() > 
//   // 	      original.at(j)->getFaceProbabilityResult() ) {
  
//   // 	    original.at( j )->setAsConsidered();  // Mark the forestWindows entry!  
  
//   // 	    temp.push_back( new UserDef::WindowClass( 
//   // 						     config.windows.at(i)->getY(), 
//   // 						     config.windows.at(i)->getX(), 
//   // 						     config.windows.at(i)->getHeight() > 
//   // 						     original.at(j)->getHeight() ? 
//   // 						     config.windows.at(i)->getHeight() : 
//   // 						     original.at(j)->getHeight(),
//   // 						     config.windows.at(i)->getWidth() > 
//   // 						     original.at(j)->getWidth() ?
//   // 						     config.windows.at(i)->getWidth() : 
//   // 						     original.at(j)->getWidth(), 
//   // 						     config.windows.at(i)->getFaceProbabilityResult() 
//   // 						      ) ); 
  
//   // 	  }		
  
//   // 	  else ; // The reference entry stays
  
//   // 	} // if( inRange.. 
//   //       } // Checked against all forestWindows 
  
  
//   //       if( !thereIsNeighbor ) 
  
//   // 	temp.push_back( new UserDef::WindowClass( 
//   // 						 config.windows.at(i)->getY(),
//   // 						 config.windows.at(i)->getX(),
//   // 						 config.windows.at(i)->getHeight(),
//   // 						 config.windows.at(i)->getWidth(),
//   // 						 config.windows.at(i)->
//   // 						 getFaceProbabilityResult() ) ); 
  
  
//   //       thereIsNeighbor = false; // Reset
  
//   //     }
  
//   //     // Set up forestWindows
//   //     config.forestWindows.clear(); 
  
//   //     // Copy temp's contents (the good config.windows members)
//   //     for( int i = 0; i < temp.size(); i++ )
  
//   //       config.forestWindows.push_back( new UserDef::WindowClass( temp.at(i)->getY(), 
//   // 								temp.at(i)->getX(),
//   // 								temp.at(i)->getHeight(),
//   // 								temp.at(i)->getWidth(),
//   // 								temp.at(i)->getFaceProbabilityResult() ) );
  
//   //     // Copy the original forestwindows members (those that have not been ousted) 
//   //     for( int i = 0; i < original.size(); i++ ) 
  
//   //       if( !original.at(i)->isConsidered() )
  
//   // 	config.forestWindows.push_back( new UserDef::WindowClass( original.at(i)->getY(),
//   // 								  original.at(i)->getX(),
//   // 								  original.at(i)->getHeight(),
//   // 								  original.at(i)->getWidth(),
//   // 								  original.at(i)->getFaceProbabilityResult()
//   // 								  ) ); 
  
//   //   }
  
// } 


/* 
 *  Dummy_combineVotes is a non-heuristic alternative to combineVotes. While combine votes 
 *  checks against past detections to put in only those that are not already there in the 
 *  neighborhood of anything before, dummy_combineVotes blindly inserts every detection
 *  from every config.windows set
 */

// /* _working_ */ 
// void dummy_combineVotes(UserDef::ConfigClass &config, int range, double scaleFactor) { 
  
//   scaleFactor = 1.0F/scaleFactor;
  
  
//   for(unsigned int i = 0 ; i < config.windows.size() ; i++ ) 
//     config.forestWindows.push_back( new UserDef::WindowClass( (int)( scaleFactor * 
// 																	(double)(config.windows.at(i)->getY()) ),
// 															 (int)( scaleFactor * 
// 																   (double)(config.windows.at(i)->getX()) ),
// 															 (int)( scaleFactor * 
// 																   (double)(config.imageInfoInstance.getHeight()) ),
// 															 (int)( scaleFactor * 
// 																   (double)(config.imageInfoInstance.getWidth()) ),
// 															 config.windows.at(i)->
// 															 getFaceProbabilityResult() ) );
  
  
// }


// /*****************************************
 
//  combineVotes - the old scheme
 
//  ****************************************/

// // void combineVotes(UserDef::ConfigClass &config , int range, double scaleFactor) { 

// //   // Since I multiplied all the terms by scaleFactor, when I insert it back , the terms are to be divided. 
// //   scaleFactor = 1.0F/scaleFactor;


// //   if(config.forestWindows.size()==0) // If the size is 0, put everything in 

// //     for(int i=0;i<config.windows.size();i++) 

// //       config.forestWindows.push_back( new UserDef::WindowClass( 
// // 							       (int)(scaleFactor * 
// // 								     config.windows.at(i)->getY()), 
// // 							       (int)(scaleFactor * 
// // 								     config.windows.at(i)->getX()), 
// // 							       (int)(scaleFactor * 
// // 								     config.imageInfoInstance.getHeight()), 
// // 							       (int)(scaleFactor * 
// // 								     config.imageInfoInstance.getWidth()), 
// // 							       config.windows.at(i)->
// // 							       getFaceProbabilityResult() ) );

// //   else {

// //     // In case the size is not 0, then we run through the set of detections we have in forestWindows. 
// //     // If we find something in the neighborhood, then we choose the best Window between 
// //     // config.windows' one and config.forestWindows' one. 
// //     // If however, for an entry of config.windows, not a single forestWindows is member, then put 
// //     // the config.windows' entry to forestWindows

// //     using Auxillary::inRange;

// //     using UserDef::WindowClass;

// //     vector< MergeWindowClass* > original; 
// //     vector< WindowClass* > temp; 

// //     for( int i = 0; i < config.forestWindows.size(); i++ ) 

// //       original.push_back( new MergeWindowClass( config.forestWindows.at(i)->getY(), 
// // 						config.forestWindows.at(i)->getX(),
// // 						config.forestWindows.at(i)->getHeight(),
// // 						config.forestWindows.at(i)->getWidth(),
// // 						config.forestWindows.at(i)->
// // 						getFaceProbabilityResult() ) ); 



// //     for(int i=0;i<config.windows.size();i++) { 

// //       bool thereIsNeighbor = false; 

// //       for(int j=0;j<original.size();j++) { 

// // 	if(	       
// // 	   inRange( (int)(scaleFactor*config.windows.at(i)->getX()), 
// // 		    original.at(j)->getX(), 
// // 		    range )  && 
// // 	   inRange( (int)(scaleFactor*config.windows.at(i)->getY()), 
// // 		    original.at(j)->getY(), 
// // 		    range ) ) { 

// // 	  thereIsNeighbor = true;  // Now, we choose the best neighbor 

// // 	  if( config.windows.at(i)->getFaceProbabilityResult() > 
// // 	      original.at(j)->getFaceProbabilityResult() ) {

// // 	    original.at( j )->setAsConsidered();  // Mark the forestWindows entry!  

// // 	    temp.push_back( new UserDef::WindowClass( 
// // 						     (int)(scaleFactor * 
// // 							   config.windows.at(i)->getY()), 
// // 						     (int)(scaleFactor * 
// // 							   config.windows.at(i)->getX()), 
// // 						     (int)(scaleFactor * 
// // 							   config.imageInfoInstance.
// // 							   getHeight()), 
// // 						     (int)(scaleFactor * 
// // 							   config.imageInfoInstance.
// // 							   getWidth()), 
// // 						     config.windows.at(i)->
// // 						     getFaceProbabilityResult() ) );

// // 	  }		

// // 	  else ; // The reference entry stays

// // 	} // if( inRange.. 
// //       } // Checked against all forestWindows 


// //       if( !thereIsNeighbor ) 
// // 	temp.push_back( new UserDef::WindowClass( 
// // 						 (int)(scaleFactor * 
// // 						       config.windows.at(i)->getY()), 
// // 						 (int)(scaleFactor * 
// // 						       config.windows.at(i)->getX()),
// // 						 (int)(scaleFactor * 
// // 						       config.imageInfoInstance.
// // 						       getHeight()),
// // 						 (int)(scaleFactor * 
// // 						       config.imageInfoInstance.
// // 						       getWidth()), 
// // 						 config.windows.at(i)->
// // 						 getFaceProbabilityResult() ) );


// //       thereIsNeighbor = false; // Reset

// //     }

// //     // Set up forestWindows
// //     config.forestWindows.clear(); 

// //     // Copy temp's contents (the good config.windows members)
// //     for( int i = 0; i < temp.size(); i++ )

// //       config.forestWindows.push_back( new UserDef::WindowClass( temp.at(i)->getY(), 
// // 								temp.at(i)->getX(),
// // 								temp.at(i)->getHeight(),
// // 								temp.at(i)->getWidth(),
// // 								temp.at(i)->getFaceProbabilityResult() ) );

// //     // Copy the original forestwindows members (those that have not been ousted) 
// //     for( int i = 0; i < original.size(); i++ ) 

// //       if( !original.at(i)->isConsidered() )

// // 	config.forestWindows.push_back( new UserDef::WindowClass( original.at(i)->getY(),
// // 								  original.at(i)->getX(),
// // 								  original.at(i)->getHeight(),
// // 								  original.at(i)->getWidth(),
// // 								  original.at(i)->getFaceProbabilityResult()
// // 								  ) ); 

// //   }


// // }



// /*****************************************
 
//  comparePerformance
 
//  ****************************************/

// // void comparePerformance(UserDef::ConfigClass &config, ifstream &in, int range) { 

// //   vector<int> x;
// //   vector<int> y;

// //   int tempx, tempy;
// //   using Auxillary::inRange;

// //   int numberOfDetections = config.forestWindows.size();
// //   int  hits = 0;
// //   int missed = 0;
// //   int numberOfFaces = 0;


// //   if(in.is_open())

// //     while(!in.eof()) { 

// //       in>>tempx;
// //       in>>tempy;
// //       x.push_back(tempx);
// //       y.push_back(tempy);
// //     }

// //   numberOfFaces = x.size();

// //   if(numberOfDetections == 0) { 

// //     cout<<"tp: "<<0<<"fp: "<<0<<"missed: "<<x.size()<<endl; return; 
// //   } 

// //   for(int i=0;i<config.forestWindows.size();i++)

// //     for(int j=0;j<numberOfFaces;j++)

// //       if( inRange(config.forestWindows.at(i)->getX(), x.at(j), range) 
// // 	  && 
// // 	  inRange(config.forestWindows.at(i)->getY(), y.at(j), range) 
// // 	  ) { 

// // 	hits++;

// // 	if(j==0) { 
// // 	  x.erase(x.begin()); y.erase(y.begin()); 
// // 	}

// // 	else { 
// // 	  x.erase(x.begin()+j-1, x.begin()+j); y.erase(y.begin()+j-1, y.begin()+j); 
// // 	} 

// // 	break;
// //       }

// //   cout<<"tpr: "<<hits/numberOfFaces
// //       <<" fp: "<<numberOfDetections - numberOfFaces
// //       <<" missed: "<<numberOfFaces - hits<<endl;

// // }



// void getForestDetections( UserDef::ConfigClass& config,
// 						 int forestSize,
// 						 vector< Forest::Tree* > &forest, 
// 						 int *category, 
// 						 int indexOfFaceCategory, 
// 						 int dt,
// 						 string nameLocOfImage, 
// 						 bool saveDetected, 
// 						 IplImage* imageToResize, 
// 						 double* scales,
// 						 int number_of_scale_ranges, 
// 						 Forest::Tree* aTree,
// 						 std::ofstream& detections_dat
// 						 ) {
  
  
//   using Forest::Tree;
//   using UserDef::WindowClass;
  
  
//   // By default the individual detections and the final resulting detection are all SAVED
//   bool saveIndividualTreeHits = true; 
//   bool saveFinalForestHits = true; 
  
//   if( !saveDetected ) { 
//     saveIndividualTreeHits = false;
//     saveFinalForestHits = false; 
//   }
  
//   //  bool testImageSupplied = false; 
//   //  bool treeIsGrown = false; 
  
//   //  Forest::Tree* aTree = 0;
//   int numberOfTreesGrown = 0 ;
  
  
//   while( numberOfTreesGrown < forestSize ) { 
    
//     // The handling of the performance of a forest will be handleded individually 
//     // depenending on whether its (a) Classification (b) Sliding window detection 
    
//     using UserDef::WindowClass; 
//     WindowClass* wc = 0 ;
//     for( unsigned int i = 0 ; i < config.forestWindows.size() ; i++ ) { 
//       wc = config.forestWindows.at( i ); 
//       delete wc; 
//     }
	
//     config.forestWindows.clear();
//     numberOfTreesGrown++; 
	
	
//     //   if( aTree != 0 ) 
// 	// 	dealloc_mem( aTree ); 
	
	
	
// 	//       aTree = new Forest::Tree( config, 
// 	// 				globalNodeIndex, 
// 	// 				numberOfTreesGrown++ ); // 1. First grow the tree
	
	
	
// 	for( int nos_scales = 0 ; nos_scales < number_of_scale_ranges ; nos_scales++ ) { 
	  
// 	  UserDef::WindowClass* wc = 0 ;
// 	  for( unsigned int i = 0 ; i < config.windows.size() ; i++ ) { 
// 		wc = config.windows.at( i ) ; 
// 		delete wc; 
// 	  }
	  
	  
// 	  config.windows.clear(); 
	  
// 	  //	string nameLocOfImage;
	  
// 	  // 	if( testImageSupplied ) 
// 	  // 	  nameLocOfImage = testImagePath; 
	  
// 	  // 	else
// 	  // 	  nameLocOfImage = TEST_IMAGE_SLIDING_WINDOW;
	  
// 	  double scaleFactor = scales[ nos_scales ]; 
	  
// 	  // 	  IplImage* imageToResize = UserDef::getImageFromLocation(nameLocOfImage);
	  
// 	  config.setDetectionThreshold((int)( scaleFactor * dt ) );
	  
	  
// 	  /* I am already doing subwindow scaling later on, why am I doing it again here? 
	   
// 	   int sw = config.getSubwindowRange(); 
// 	   sw = (int)( scaleFactor * sw ); 
	   
// 	   if( sw > 4 ) 
// 	   config.setSubwindowRange( sw ); 
	   
	   
// 	   */
	  
// 	  bool security_check = false; 
	  
// 	  /* If size goes beyond 1400, noticed a problem
// 	   */
	  
// 	  if( (int)(scaleFactor * imageToResize->height) < 1300 && 
// 		 (int)(scaleFactor * imageToResize->width) < 1300 ) { 
		
// 		if( security_check ) { 
		  
// 		  if(config.getDetectionThreshold()==0 || config.getDetectionThreshold()<0)
// 			config.setDetectionThreshold(1);
		  
// 		  if( 
// 			 config.imageInfoInstance.getHeight()  >
// 			 (int)( scaleFactor * config.testImageInfoInstance.getHeight() ) ||
// 			 config.imageInfoInstance.getWidth()  >
// 			 (int)( scaleFactor * config.testImageInfoInstance.getWidth() ) 
// 			 )
// 			scaleFactor = 1.0;  // Is scale too magnified? Then, do not resize test Image
		  
// 		  if(
// 			 (int)( scaleFactor * config.testImageInfoInstance.getHeight() ) == 0  
// 			 || 
// 			 (int)( scaleFactor * config.testImageInfoInstance.getWidth() ) == 0 
// 			 )
// 			scaleFactor = 1.0; // Is scale too minute? Then, do not resize test image
		  
		  
// 		}
		
// 		CvSize resizeImageSize = cvSize( (int)(scaleFactor*imageToResize->width), 
// 										(int) (scaleFactor*imageToResize->height) );
		
// 		IplImage* slidingWindowTestIplImage = cvCreateImage( resizeImageSize, 
// 															imageToResize->depth, 
// 															imageToResize->nChannels );
		
// 		cvResize( imageToResize, 
// 				 slidingWindowTestIplImage, 
// 				 CV_INTER_LINEAR );
		
		
// 		int* integralImageOfSlidingWindow = new int[( slidingWindowTestIplImage->height) * 
// 													(slidingWindowTestIplImage->width)];
		
// 		using UserDef::calculateIntegralImage; 
// 		calculateIntegralImage(slidingWindowTestIplImage, 
// 							   integralImageOfSlidingWindow);
		
// 		config.testImageInfoInstance.setHeight(slidingWindowTestIplImage->height); //set new height
// 		config.testImageInfoInstance.setWidth(slidingWindowTestIplImage->width); //set new widht
		
// 		int status; 
		
// 		status = aTree->slidingWindow( aTree, 
// 									  integralImageOfSlidingWindow, 
// 									  config, 
// 									  category, 
// 									  indexOfFaceCategory, 
// 									  aTree->getForestIndex(), 
// 									  scaleFactor 
// 									  );
		
		
		
// 		if( status == 0 ) { 
		  
// 		  config.globalDetectionsFile<<"\n---------End of Tree: Before Merges------------\n";
// 		  Auxillary::printConfigWindows( config, config.windows ); 
		  
		  
// 		  config.globalDetectionsFile<<"\n---------End of Tree: After Merges------------\n";      
		  
// 		  mergeDetections( config.windows, config , scaleFactor ); 
		  
// 		  Auxillary::printConfigWindows( config, config.windows ); 
		  
		  
// 		  //	  if( forestSize > 1 ) {  // integrate the results of each tree
		  
// 		  dummy_combineVotes(config, 
// 							 NEIGHBOR_RANGE, 
// 							 scaleFactor ); 
// 		  //	  } 
		  
		  
// 		  if( saveIndividualTreeHits ) { 
// 			char nameAdd[ 10 ] ;
// 			sprintf( nameAdd , "%d#%d-" , numberOfTreesGrown , nos_scales );  // Unique name
// 			UserDef::drawDetections( slidingWindowTestIplImage, 
// 									config.windows, 
// 									nameLocOfImage, 
// 									nameAdd, 
// 									saveDetected );
// 		  }
// 		}
// 		cvReleaseImage( &slidingWindowTestIplImage );
// 		delete[] integralImageOfSlidingWindow; 
		
// 	  }      
      
// 	} // for( nos_scales .. 
	
// 	config.globalDetectionsFile<<"\n---GLOBAL DETECTIONS-BEFORE TRIM--\n"; 
// 	Auxillary::printConfigWindows( config, config.forestWindows );   
	
	
// 	using UserDef::filterNoise5; 
// 	filterNoise5( config.forestWindows, config, 1.0, 0 ); 
	
	
	
	
// 	config.globalDetectionsFile<<"\n---GLOBAL DETECTIONS-AFTER-TRIM--\n"; 
// 	Auxillary::printConfigWindows(config, config.forestWindows); 
	
	
// 	if( saveFinalForestHits ) { 
	  
// 	  CvSize resizeImageSize = cvSize( imageToResize->width, imageToResize->height ); 
	  
// 	  IplImage* slidingWindowTestIplImage = cvCreateImage( resizeImageSize, 
// 														  imageToResize->depth, 
// 														  imageToResize->nChannels );
	  
// 	  cvResize( imageToResize, 
// 			   slidingWindowTestIplImage, 
// 			   CV_INTER_LINEAR );
      
      
// 	  char nameAdd[ 20 ];
// 	  sprintf( nameAdd, "final_%d-", numberOfTreesGrown ); 
      
// 	  UserDef::drawDetections(slidingWindowTestIplImage, 
// 							  config.forestWindows, 
// 							  nameLocOfImage,
// 							  nameAdd, 
// 							  saveDetected ); 
	  
	  
// 	  UserDef::WindowClass* theWindow;
// 	  //Write detections down 
// 	  char *buffer = new char[400]; 
// 	  char *buffer_name = new char[PATH_MAX + 400];
// 	  char* temp = new char[20];
// 	  for( unsigned int i = 0; i<config.forestWindows.size(); i++ ) 
// 	  { 
// 	    theWindow = config.forestWindows.at(i);
// 	    sprintf( temp, " %d %d %d %d ", theWindow->getX(), theWindow->getY(), theWindow->getWidth(), theWindow->getHeight() );
	    
// 	    if( i == 0 )
// 	      strcpy( buffer, temp ); 
// 	    else
// 	      strcat( buffer, temp ); 
// 	  }
	  
// 	  cout << buffer; 
// 	  strcpy(buffer_name, nameLocOfImage.c_str());
// 	  strcat(buffer_name, buffer); 
	  
// 	  detections_dat << buffer_name << endl;  // Written
	  
	  
// 	  delete[] buffer;
// 	  delete[] buffer_name; 
	  
// 	  cvReleaseImage(&slidingWindowTestIplImage);
	  
// 	}      
//   } // while 
  
  
  
  
  
//   //   static double scales[] = {1.0, 0.25 , 0.5, 0.8 , 0.3, 0.6 , 0.2, 0.4 , 0.1};
  
//   //   static bool forestGrown = false; 
  
//   //   int globalNodeIndex = 0;
  
//   //   if( !forestGrown ) { 
  
//   //     for( int i = 0; i < forestSize; i++ ) { 
  
//   //       Tree* aTree = new Tree( config,  
//   // 			      globalNodeIndex, 
//   // 			      i );
  
//   //       forest.push_back( aTree ); 
//   //       globalNodeIndex = 0; 
  
//   //     }
//   //   } 
  
//   //   forestGrown = true; 
  
  
//   //   config.forestWindows.clear(); // A set of detections for each image should be collected 
  
//   //   for( int f = 0; f < forest.size(); f++) { 
  
//   //     double scaleFactor = scales[ f ]; 
//   //     Tree* aTree = forest.at( f ); 
  
//   //     config.windows.clear(); 
  
//   //     config.setDetectionThreshold((int)( scaleFactor * dt ) );
  
//   //     if(config.getDetectionThreshold()==0 || config.getDetectionThreshold()<0)
  
//   //       config.setDetectionThreshold(1);
  
  
//   //     CvSize resizeImageSize = cvSize( (int)(scaleFactor*imageToResize->width), 
//   // 				     (int) (scaleFactor*imageToResize->height) );
  
//   //     IplImage* slidingWindowTestIplImage = cvCreateImage( resizeImageSize, 
//   // 							 imageToResize->depth, 
//   // 							 imageToResize->nChannels );
  
//   //     cvResize( imageToResize, 
//   // 	      slidingWindowTestIplImage, 
//   // 	      CV_INTER_LINEAR );
  
//   //     //    cvReleaseImage( &imageToResize );
  
//   //     int* integralImageOfSlidingWindow = new int[( slidingWindowTestIplImage->height) * 
//   // 						(slidingWindowTestIplImage->width)];
  
//   //     calculateIntegralImage(slidingWindowTestIplImage, 
//   // 			   integralImageOfSlidingWindow);
  
//   //     config.testImageInfoInstance.setHeight(slidingWindowTestIplImage->height); //set config details
//   //     config.testImageInfoInstance.setWidth(slidingWindowTestIplImage->width); //set config details
  
//   //     aTree->slidingWindow( aTree, 
//   // 			  integralImageOfSlidingWindow, 
//   // 			  config, 
//   // 			  category, 
//   // 			  indexOfFaceCategory, 
//   // 			  aTree->getForestIndex() );
  
  
  
//   //     Auxillary::printConfigWindows( config, config.windows ); 
//   //     config.globalDetectionsFile<<"\n---------End of Tree: Before Merges------------\n";
  
//   //     mergeDetections( config.windows, config );
  
//   //     Auxillary::printConfigWindows( config, config.windows ); 
//   //     config.globalDetectionsFile<<"\n---------End of Tree: After Merges------------\n";
  
//   //     if( forest.size() > 1 ) {  // integrate the results of each tree
  
  
//   //       combineVotes(config, 
//   // 		   NEIGHBOR_RANGE, 
//   // 		   scaleFactor ); 
//   //     } 
  
//   //     char nameAdd[10];
//   //     sprintf(nameAdd,"%d-", f);
  
//   //     if( saveIndividualTreeHits )
//   //       UserDef::drawDetections( slidingWindowTestIplImage, 
//   // 			       config.windows, 
//   // 			       nameLocOfImage, 
//   // 			       nameAdd, 
//   // 			       saveIndividualTreeHits );
  
//   //     cvReleaseImage( &slidingWindowTestIplImage );
  
//   //   }
  
//   //   if(forestSize>1){ 
  
//   //     //Draw the final result of all detections
  
//   //     config.globalDetectionsFile<<"\n---GLOBAL DETECTIONS-BEFORE TRIM--\n"; 
//   //     Auxillary::printConfigWindows( config, config.forestWindows );   
  
//   //     using UserDef::filterNoise2; 
//   //     filterNoise2( config, config.forestWindows);
  
//   //     IplImage* slidingWindowTestIplImage = UserDef::getImageFromLocation(testImagePath);
//   //     char *nameAdd = "FINAL-";
  
//   //     config.globalDetectionsFile<<"\n---GLOBAL DETECTIONS-AFTER-TRIM--\n"; 
//   //     Auxillary::printConfigWindows(config, config.forestWindows); 
  
//   //     if( saveFinalForestHits ) 
//   //       UserDef::drawDetections(slidingWindowTestIplImage, 
//   // 			      config.forestWindows, 
//   // 			      nameLocOfImage, 
//   // 			      nameAdd, 
//   // 			      saveFinalForestHits);
  
//   //     cvReleaseImage(&slidingWindowTestIplImage);
//   //   }	
  
  
  
// }


// void checkClassification( char* infoname ) 
// {
  
//   FILE* info;
//   char fullname[PATH_MAX];
//   char* filename; 
//   IplImage* img; 
  
//   strcpy( fullname, infoname );
//   filename = strrchr( fullname, '\\' );
//   if( filename == NULL )
//   {
// 	filename = strrchr( fullname, '/' );
//   }
//   if( filename == NULL )
//   {
// 	filename = fullname;
//   }
//   else
//   {
// 	filename++;
//   }
  
//   info = fopen( infoname, "r" );
//   if( info != NULL )
//   {
	
// 	while( !feof( info ) )
// 	{
	  
	  
// 	  img = cvLoadImage( fullname );
	  
// 	  if( !img ) continue; 
	  
// 	  cvReleaseImage( &img );
// 	}
//   }
// }


// void runPerformance(UserDef::ConfigClass& config,  char* infoname, 
// 					int *category, 
// 					int indexOfFaceCategory, 
// 					int saveDetected, 
// 					int dt, 
// 					int forestSize, 
// 					float maxPosDiff,
// 					float maxSizeDiff, 
// 					double* scales,
// 					int nos_scales, 
// 					bool roc_mode ) { 
  
  
  
//   ofstream detections_dat;
//   detections_dat.open( FINAL_DETECTIONS ); 
  
//   /* 
//    * the roc_mode variable is going to turn off irrelevant and irritating output produced from detection 
//    * accuracy module
//    */
  
  
//   int i, j;
  
//   //  bool theActualImplementation = true;
  
//   //  char* samplesdir    = NULL;
  
//   vector< Forest::Tree* > forest; 
  
  
// //   int width  = 24;
// //   int height = 24;
  
// //   double scale_factor = 1.2;
  
  
//   FILE* info;
  
//   char fullname[PATH_MAX];
// //   char detfilename[PATH_MAX];
//   char* filename;
// //   char detname[] = "det-";
  
  
//   CvMemStorage* storage;
// //   CvSeq* objects;
  
//   double totalTime;
  
  
//   Forest::Tree* aTree = 0 ;
//   int globalNodeIndex = 0 ; 
  
//   // The Tree 
//   aTree = new Forest::Tree( config , 
// 						   globalNodeIndex , 
// 						   1 ); 
  
//   if( !roc_mode )
//     cout<<infoname;
  
//   storage = cvCreateMemStorage();
//   strcpy( fullname, infoname );
  
//   filename = strrchr( fullname, '\\' );
//   if( filename == NULL )
//   {
// 	filename = strrchr( fullname, '/' );
//   }
//   if( filename == NULL )
//   {
// 	filename = fullname;
//   }
//   else
//   {
// 	filename++;
//   }
  
//   info = fopen( infoname, "r" );
//   totalTime = 0.0;
  
//   if( info != NULL )
//   {
// 	int x, y, width, height;
// 	IplImage* img;
// 	int hits, missed, falseAlarms;
// 	int totalHits, totalMissed, totalFalseAlarms;
// 	int found;
// 	float distance;
	
// 	int refcount;
// 	ObjectPos* ref;
// 	//int detcount;
// 	ObjectPos* det;
// 	int error;
	
// 	int* pos;
// 	int* neg;
	
// 	pos = (int*) cvAlloc( forestSize * sizeof( *pos ) );
// 	neg = (int*) cvAlloc( forestSize * sizeof( *neg ) );
// 	for( i = 0; i < forestSize; i++ ) { pos[i] = neg[i] = 0; }
	
	
// 	if( !roc_mode ) { 
// 	  printf( "+================================+======+======+======+\n" );
// 	  printf( "|            File Name           | Hits |Missed| False|\n" );
// 	  printf( "+================================+======+======+======+\n" );
// 	}
	
// 	totalHits = totalMissed = totalFalseAlarms = 0;
// 	while( !feof( info ) )
// 	{
// 	  if( fscanf( info, "%s %d", filename, &refcount ) != 2 || refcount <= 0 ) break;
	  
// 	  img = cvLoadImage( fullname );
	  
// 	  ref = (ObjectPos*) cvAlloc( refcount * sizeof( *ref ) );
// 	  if( !img ) continue;
	  
	  
	  
// 	  for( i = 0; i < refcount; i++ )
// 	  {
		
// 		error = (fscanf( info, "%d %d %d %d", &x, &y, &width, &height ) != 4);
// 		if( error ) break;
// 		ref[i].x = 0.5F * width  + x;
// 		ref[i].y = 0.5F * height + y;
// 		ref[i].width = sqrtf( 0.5F * (width * width + height * height) );
// 		ref[i].found = 0;
// 		ref[i].neighbors = 0;
// 	  }
// 	  if( !error ){
	    
// 	    cvClearMemStorage( storage );
	    
	    
// 	    totalTime -= time( 0 );
// 		// 	    int* integralImageOfSlidingWindow = new int[(img->height)*(img->width)];
// 		// 	    config.testImageInfoInstance.setWidth(img->width);
// 		// 	    config.testImageInfoInstance.setHeight(img->height);
// 		// 	    calculateIntegralImage(img, integralImageOfSlidingWindow);
// 	    config.windows.clear(); 
	    
// 	    getForestDetections(config,
// 							forestSize,
// 							forest, 
// 							category, 
// 							indexOfFaceCategory,
// 							dt,
// 							fullname, 
// 							saveDetected, 
// 							img,
// 							scales,
// 							nos_scales, 
// 							aTree, 
// 							detections_dat);
	    
// 	    cvReleaseImage( &img ); 
		
// 		//   printf("The forestWindow size is :%d\n", config.forestWindows.size() ); 
	    
	    
// 	    // 	    cout<< "RandomTree@runPerformance: Config.forestwindows.size() : " <<
// 	    // 	      config.forestWindows.size() <<
// 	    // 	      endl;
	    
		
// 		// 	    UserDef::WindowClass* wc = 0; 
// 		// 	    for( int i = 0 ; i < config.windows.size() ; i++ ) { 
// 		// 	      wc = config.windows.at( i );
// 		// 	      delete wc; 
// 		// 	    }
	    
// 		// 	    config.windows.clear();
		
// 		// 	    for( int q = 0; q < config.forestWindows.size(); q++ ) 
// 		// 	      config.windows.push_back( config.forestWindows.at( q ) ); 
		
		
// 		// 	    config.forestWindows.clear(); 
		
// 	    // 	    cout<< "RandomTree@runPerformance: Config.windows.size() : " <<
// 	    // 	      config.windows.size() <<
// 	    // 	      endl;
	    
// 	    //	    aTree->slidingWindow(aTree, integralImageOfSlidingWindow, 
// 	    // config, category, indexOfFaceCategory, aTree->getForestIndex());
	    
// 	    totalTime += time( 0 );
	    
// 	    det = (ObjectPos*) cvAlloc( config.forestWindows.size() * sizeof( *det ) );
// 	    hits = missed = falseAlarms = 0;
// 	    for( unsigned int i = 0; i < config.forestWindows.size(); i++ ) {
	      
// 	      det[i].x = 0.5F * config.forestWindows.at(i)->getWidth()  + config.forestWindows.at(i)->getX();
// 	      det[i].y = 0.5F * config.forestWindows.at(i)->getHeight() + config.forestWindows.at(i)->getY();
// 	      det[i].width = sqrtf( 0.5F * (config.forestWindows.at(i)->getHeight() * 
// 										config.forestWindows.at(i)->getWidth() + 
// 										config.forestWindows.at(i)->getWidth() * 
// 										config.forestWindows.at(i)->getHeight()) );
// 	      //		det[i].neghbors = r.neighbors;
// 	      det[i].neighbors = 0;
	      
// 	      //		if(theActualImplementation)
// 	      for( j = 0; j < refcount; j++ ) { 
			
			
// 			found = 0; 
// 			distance = sqrtf( (det[i].x - ref[j].x) * (det[i].x - ref[j].x) + 
// 							 (det[i].y - ref[j].y) * (det[i].y - ref[j].y) );
// 			if( (distance < ref[j].width * maxPosDiff) &&
// 			   (det[i].width > ref[j].width / maxSizeDiff) &&
// 			   (det[i].width < ref[j].width * maxSizeDiff) )
// 			{
// 			  ref[j].found = 1;
// 			  ref[j].neighbors = MAX( ref[j].neighbors, det[i].neighbors );
// 			  found = 1;
// 			}
			
// 			//		  } /* Not actual implementation */
// 			// else {
// 			// 		  using Auxillary::inRange;
// 			// 		  for(j=0;j<refcount;j++) { 
// 			// 		    int rangeCheck =  config.getDetectionThreshold(); 
// 			// 		    rangeCheck = 5;
// 			// 		    if( 
// 			// 		       inRange((int)det[i].x,(int) ref[j].x,rangeCheck)
// 			// 		       && 
// 			// 		       inRange((int)det[i].y, (int)ref[j].y, rangeCheck)
// 			// 		       )
// 			// 		      {
// 			// 			ref[j].found = 1;
// 			// 			ref[j].neighbors = MAX( ref[j].neighbors, det[i].neighbors );
// 			// 			found = 1;
// 			// 		      }
// 			// 		  }
// 			// 		}
			
// 			if( !found )
// 			{
// 			  falseAlarms++;
// 			  neg[MIN(det[i].neighbors, forestSize - 1)]++;
// 			} 
// 	      } 
	      
// 	    }
	    
// 	    //	    cout<<"RandomTree@runPerformance: falseAlarms"<<falseAlarms<<endl;
// 	    for( j = 0; j < refcount; j++ )
// 		{
// 		  if( ref[j].found )
// 		  {
// 		    hits++;
// 		    pos[MIN(ref[j].neighbors, forestSize - 1)]++;
// 		  }
// 		  else
// 		  {
// 		    missed++;
// 		  }                            
// 		}
		
		
// 	    totalHits += hits;
// 	    totalMissed += missed;
// 	    totalFalseAlarms += falseAlarms;
		
// 	    if( !roc_mode) { 
// 		  printf( "|%32.32s|%6d|%6d|%6d|\n", filename, hits, missed, falseAlarms );
// 		  printf( "+--------------------------------+------+------+------+\n" );
// 	    }
		
// 	    fflush( stdout );
		
	    
		
// 		// 	    if( saveDetected )
// 		// 	      {
// 		// 		strcpy( detfilename, detname );
// 		// 		strcat( detfilename, filename );
// 		// 		strcpy( filename, detfilename );
// 		// 		cvvSaveImage( fullname, img );
// 		// 	      }
		
// 	    if( det ) { cvFree( &det ); det = NULL; }
	    
// 	    UserDef::WindowClass* temp;
// 	    for( unsigned int i = 0 ; i < config.forestWindows.size(); i++ )  { 
// 	      temp = config.forestWindows.at( i ) ; 
// 	      delete temp; 
// 	    }
	    
// 	    config.forestWindows.clear(); 
	    
// 	  } /* if( !error ) */
	  
// 	  //	  cvReleaseImage( &img );
// 	  cvFree( &ref );
// 	}
	
	
	
	
// 	fclose( info );
	
	
// 	if( !roc_mode ) { 
//       printf( "|%32.32s|%6d|%6d|%6d|\n", "Total",
// 			 totalHits, totalMissed, totalFalseAlarms );
//       printf( "+================================+======+======+======+\n" );
//       printf( "Total time: %f\n", totalTime );
// 	}
	
// 	for( i = forestSize - 1; i > 0; i-- )
// 	{
// 	  pos[i-1] += pos[i];
// 	  neg[i-1] += neg[i];
// 	}
	
	
// 	if( roc_mode ) { 
// 	  /*
// 	   * Viola and Jones (http://www.stat.uchicago.edu/~amit/19CRS/DEA/cascade_face_detection.pdf) defines the total number of false subwindows
// 	   * as 350 million or 350 x 10^6 or 350000000. I am going use this to define the fpr. 
// 	   * The tpr is as defined : positive_hits / total_number_of_face
// 	   */ 
	  
	  
// 	  /* 
// 	   * This is the output according to the true definition of ROC
// 	   */
// 	  //float tpr = ( 1.0 * totalHits)/ ( 1.0 * ( totalHits + totalMissed ) ); 
// 	  //	float fpr = ( 1.0 * totalFalseAlarms )/350000000.00; 
// 	  //	fprintf(stderr,"\n%f %f\n", tpr, fpr ); 	
	  
	  
// 	  /* 
// 	   * ROC results as per viola and jones output method 
// 	   */
	  
// 	  fprintf( stderr,"%f %f\n", 
// 			  ((float)totalHits/(totalHits+totalMissed)) , 
// 			  ((float)totalFalseAlarms/(totalHits+totalMissed)) ); 
	  
// 	}
	
// 	/* print ROC to stdout */
// 	//       for( i = 0; i < forestSize; i++ )
// 	//         {
	
// 	/* Alternate defintion of ROC */
// 	/* 
// 	 * this is the roc results for viola and jones. It represents the output for each stage.
// 	 * So, in effect, the algorithm considers the output across different stages.
// 	 * pos[0] gives the number of positives as generated by the first stage.
// 	 * pos[1] gives the same for stage 2, but obviously the total number of positive hits = those by pos[0] + those by pos[1]
// 	 * similarly, neg[1] is the negatives by neg[0] + that by neg[1]
// 	 * Thus, the output below can straightaway be plotted for a ROC curve. 
// 	 */
	
	
// 	// 	  fprintf( stderr, "\t%d\t%d\t%f\t%f\n", pos[i], neg[i],
// 	// 		   ((float)pos[i]) / (totalHits + totalMissed),
// 	//		   ((float)neg[i]) / (totalHits + totalMissed) );
	
// 	// 	}
	
// 	cvFree( &pos );
// 	cvFree( &neg );
//   }
  
  
  
//   delete aTree; 
//   cvReleaseMemStorage( &storage );
//   detections_dat.close();
  
  
// } 




int main( int argc, char** argv ) {
  

  
//   std::clock_t start, finish, start_detection, end_detection; // time to complete training and execution
//   start = std::clock();
  
  
  /* The set of default values */  
  bool performance = false;
  char infoname[PATH_MAX];
  char identityfile[PATH_MAX]; 
  bool identityfileSupplied = false; 
  int forestSize = 0; 
  bool run_classification = RUN_CLASSIFICATION; 
  bool relearn = false;
  int nclasses = NCLASSES; 
  bool verbose = true; 
  char trainImages[PATH_MAX]; 
  bool trainLocGiven = false; 
  float maxSizeDiff = 1.5F, maxPosDiff = 0.5F; 
  double fmp = FACE_MAP_THRESHOLD;
  int dt = DETECTION_THRESHOLD; 
  int vote = 1; 
  char testImagePath[PATH_MAX];
  bool testImageSupplied = false;
  bool saveDetected = true; // Default - all detections are saved
  int subwindow_range = SUBWINDOW_RANGE; 
  //  int number_of_scale_ranges = NOS_SCALES; 
  int save_train = false; 
  bool test_mode = false; 
  bool roc_mode = false; // Roc curves mode, output is minimal 
  
  
  
  
  if( argc==1 ) { 
	printf("\nUSAGE:%s [arg] [value]\n -relearn <1(true)/0(false)>\n"
		   "-data <Location_of_train_Images>\n"
		   "-trees <number_of_extremely_random_trees>\n\n"
		   "-fmp <face_map_threshold_double_value>\n"
		   "-dt <detection_threshold_value>\n"
		   "-nclasses <number of classes expected>\n"
		   "-classification <1(true)_0(false)>\n"
		   "-subwindow_range <range_of_subwindows_to_run_for_sliding_window>\n\n"
		   "DETECTION SCHEME:\n"
		   "-vote <1_2_3 Sliding Window Preferences>\n\n"
		   "-info <location_of_test_image>\n\n"
		   "RUN TYPE:\n"
		   "-performance <location_of_marked_images>\n"
		   "\t -maxSizeDiff <max_size_diff = 1.50000> \n"
		   "\t -maxPosDiff <max_position_diff = 1.2000> \n\n"
		   "-roc_mode \t enable roc_mode\n"
		   "-identity \t <identity_file>\n"
		   "MISC:\n"
		   "-voff \t <switch_off_verbose_mode>\n"
		   "-st \t <save_train> \n"
		   "-ni  \t add parameter if no image to be saved\n\n" , argv[0]); 
	
	return 0;
  } // if( argc==1 )..
  
  if( strcmp( argv[1],"-relearn" ) ) { 
    printf( "Please enter relearn status as 1(true) or 0(false). " );
    return 0;
  }
  
  else{ 
    int relearnStatus = atoi(argv[2]);
    if( relearnStatus==1 ) relearn = true;
    else relearn = false;
  }
  
  if( argc > 2 ){ 
	
    for( int i = 3; i < argc; i++ ) { 
      
      if( !strcmp( argv[i],"-fmp" ) ) fmp = atof(argv[++i]); 
	  
      else if( !strcmp( argv[i], "-roc_mode" ) ) roc_mode = true; 
	  
      else if( !strcmp( argv[i], "-subwindow_range" ) ) subwindow_range = atoi( argv[++i] ); 
      
      else if( !strcmp( argv[i],"-dt" ) ) dt = atoi(argv[++i]);
	  
      else if( !strcmp( argv[i], "-info" ) ) { 
		strcpy( testImagePath, argv[++i] );
		testImageSupplied = true;
      }
	  
      else if( !strcmp( argv[i], "-data" ) ) { 
		strcpy( trainImages, argv[++i] ); 
		trainLocGiven = true; 
      }
	  
	  else if( !strcmp( argv[i], "-classification" ) ) {
		int classification_value = atoi ( argv[++i] ); 
		if( classification_value == 1 ) run_classification = true; 
		else run_classification = false; 
	  }
	  
	  else if( !strcmp( argv[i], "-nclasses" ) ) nclasses = atoi( argv[++i] ); 
	  
      else if( !strcmp( argv[i], "-st" ) ) save_train = true;
	  
      else if( !strcmp( argv[i], "-test" ) ) test_mode = true;
	  
	  else if( !strcmp( argv[i], "-identity" ) ) {
		strcpy( identityfile, argv[++i] ); 
		identityfileSupplied = true; 
	  }
	  
      else if( !strcmp( argv[i], "-voff") ) verbose = false;
	  
      else if( !strcmp( argv[i], "-trees" ) ) forestSize = atoi(argv[++i]); 
      
      else if( !strcmp( argv[i], "-vote" ) ) vote = atoi(argv[++i]);
	  
      else if( !strcmp( argv[i], "-ni" ) ) saveDetected = false;
	  
      else if( !strcmp( argv[i], "-maxPosDiff" ) ) maxPosDiff = atof( argv[++i] );
      
      else if( !strcmp( argv[i], "-maxSizeDiff" ) ) maxSizeDiff = atof( argv[++i] ); 
      
      else if( !strcmp( argv[i],"-performance" ) ) { 
		performance = true;
		strcpy( infoname, argv[++i] ); 
      }
	  
    } // for(..
  } // if(argc>2)..
  

  int t_height, t_width; 
  
  ForestConfigClass config( nclasses ); 
  //UserDef::ConfigClass config( nclasses );
  IplImage* imageToClassify = cvLoadImage( testImagePath );  //UserDef::getImageFromLocation( testImagePath ); 
  if( !imageToClassify ) 
    {
      perror("qa_randomtree.cpp: image not found");
      exit(1);
    }
    
  if( forestSize == 0 )  //default forest size
    forestSize = _FOREST;


 /*
  Forest::Tree* bTree; 
  Forest::Tree* cTree; 

  growTreeFromImages( trainImages, nclasses, bTree, t_height, t_width, config ); 
 
 printf("\n %d is the class label for the image. \n", Forest::returnClassLabelForIplImage( bTree, imageToClassify, t_height, t_width, nclasses ) ); 
  
  int _globalNodeIndex = 0;
  int _numberOfTrees = 1; 
  cTree = new Tree( config, _globalNodeIndex, _numberOfTrees ); 
  printf("\n %d is the class label for the image. \n", Forest::returnClassLabelForIplImage( cTree, imageToClassify, t_height, t_width, nclasses ) ); 
  
  */
   
   //  vector< Forest::Tree* > _forest; 


   
    //UserDef::ConfigClass config( nclasses );    
  ForestClass *theForest = new ForestClass( trainImages, nclasses, t_height, t_width, config, forestSize );
   // IplImage* imageToClassify = UserDef::getImageFromLocation( testImagePath ); 
    
/*    for( int i = 0; i < _FOREST ; i++ ) 
    { 
        printf("\n %d is the forest class label for the image.\n", Forest::returnClassLabelForIplImage( theForest->getTree(i), imageToClassify, t_height, t_width, nclasses ) );   
    }
  */
    printf("\n the final label is %d.\n", get_class_label_from_forest( theForest, imageToClassify ) ); 

    delete theForest;
  
 // printf(“the final answer is %d”, Forest::getClassLabelFromForest( theForest, imageToClassify ) );
  
  
  
  
//    config.setSubwindowRange( subwindow_range ); 
  
  
//   if( !RUN_SLIDING_WINDOW && 
// 	 testImageSupplied ) 
    
//     printf("\nSYSALERT: You have supplied the test image without enabling the detection mode: \n"
// 		   "Please enable the detection mode to use this test image\n");
  
//   if( relearn )
//     config.setRelearnStatusValue( true );
//   else
//     config.setRelearnStatusValue( false );
  
//   config.setFaceMapThreshold( fmp );
//   config.setDetectionThreshold( dt );
  
  
//   int* integralImage; 
//   int *mem_chunk;
//   int *tempII; 
  
  
//   if( config.getRelearnStatusValue() ) { // learn from scratch
	
    
//     /*****************************************
	 
//      init images : Read images from Location and keep the integral Images stored 
	 
// 	 ****************************************/
    
	
//     string* s = 0;
//     s = new string( trainImages ); 
	
//     if( trainLocGiven) 
//       config.setTrainImages( *s ); 
	
//     else  
//       config.setTrainImages();
	
//     delete s; 
	
//     IplImage* tempImage = 0; 
//     DIR* dir = 0;
	
//     // Holds file name for reading images. 
//     char itoaString[20];
	
//     int i = 0, j = 0;
//     // Set to false if loaded file is not an image.
//     bool flagNotImage = false;
//     struct dirent *ent = 0;
    
//     string mainDirPath = config.getTrainImages();  
//     string fileName , dirPath;
	
	
//     /** 
//      * The object images will be populated with images from each 
//      * directory specified 
//      * for a class across all classes. ie. 
//      * images[6] is the vector of IplImages 
//      * specified for the class 6. 
//      */
//     for(i = 0;i<config.getNclasses();i++){
	  
//       j = 0;
	  
//       sprintf(itoaString,"%d",i);
	  
//       dirPath = mainDirPath + itoaString;
      
//       if((dir = opendir(dirPath.c_str())) == NULL){
// 		printf("the directory searched for is %s\n", dirPath.c_str() ); 
// 		perror("Directory does not exist");
// 		exit(0);
//       }
      
//       bool setBorders = false; 
      
//       while((ent = readdir(dir))!=NULL){
// 		fileName = dirPath + "/" + ent->d_name;
		
// 		tempImage = cvLoadImage(fileName.c_str());
		
// 		if(!tempImage){
// 		  flagNotImage = true;
// 		}
		
// 		if( !flagNotImage ) { 
// 		  j++;
		  
// 		  //	  config.images[i].push_back(tempImage);
		  
// 		  config.imageInfoInstance.setHeight( tempImage->height );
// 		  config.imageInfoInstance.setWidth( tempImage->width );
		  
// 		  tempII = new int[ ( tempImage->height ) * ( tempImage->width ) ];
		  
// 		  UserDef::calculateIntegralImage( tempImage, tempII );
		  
// 		  config.integralImages[i].iiVector.push_back( tempII );
	  	  
// 		  if( !setBorders ) { 
			
// 			config.integralImages[i].setHeight( tempImage->height ); 
// 			config.integralImages[i].setWidth( tempImage->width ); 
// 			setBorders = true; 
// 		  }
		  
		  
// 		  cvReleaseImage( &tempImage ); 
// 		} // if( !flag..  
		
// 		flagNotImage = false;
//       } // while(.. 
//     } // for( i.. 
	
	
	
//     //Write to file
//     config.setTrainIntegralImagesLoc(TRAINDT);
	
    
//     if( verbose && !roc_mode ) { 
      
	  
      
//       for( int i =0; i < config.getNclasses(); i++) 
// 		printf("Total number of training images: class %d has %zu images\n", i , config.integralImages[i].iiVector.size() );
      
//     }    
    
// 	//     UserDef::VectorOfIntegralImages vectorOfIntegralImages[ NCLASSES ];
	
// 	//     for( int  i = 0 ; i < NCLASSES ; i++ ) 
// 	//       for( int j = 0 ; j < config.integralImages[i].size() ; j++ )
// 	// 	vectorOfIntegralImages[i].iiVector.push_back( &( config.integralImages[i].iiVector.at(j) ) ); 
	
	
//     if( save_train )
//       Auxillary::writeToFile( config.integralImages, 
// 							 config.getTrainIntegralImagesLoc(), 
// 							 config.imageInfoInstance.getHeight(), 
// 							 config.imageInfoInstance.getWidth());
    
    
//     dir = 0;
//     ent = 0;
	
    
	
    
//     // ************** END initImages ************ 
	
   	
//     if( save_train )
//       Auxillary::writeToFile( config ); //Write down config file - for future use of config.getHeight() etc.  
//   } 
  
  
  
//   else{ 
    
//     /*
// 	 *************
// 	 LEARNING OFFLINE
// 	 ************
// 	 */
    
	
//     Auxillary::readFromFile( config );
    
//     config.setTrainIntegralImagesLoc( TRAINDT );
    
//     //     Auxillary::readFromFile( config.integralImages, 
//     // 			     config.getTrainIntegralImagesLoc(), 
//     // 			     config.imageInfoInstance.getHeight(), 
//     // 			     config.imageInfoInstance.getWidth() );
    
	
	
//     /************ Reading from File : Start Section ************/
	
	
//     for( int i=0;i<config.getNclasses();i++)
//       config.integralImages[i].iiVector.clear();
	
//     int height = config.imageInfoInstance.getHeight(); 
//     int width = config.imageInfoInstance.getWidth();
	
//     string readLoc = config.getTrainIntegralImagesLoc(); 
	
//     //Read the file
//     std::ifstream in;
	
//     //    in.open(readLoc.c_str());
	
//     // temp variables to hold values read from file
//     int size;
//     int var;
    
//     for(int i=0;i<config.getNclasses();i++){ //for each vector
	  
//       int p = i; 
//       char s[ 50 ]; 
//       sprintf( s, "%s-class_%d.dat", readLoc.c_str(), p ); 
	  
      
//       in.open( s ); 
//       size = 0; 
//       in>>size;
	  
      
//       //      mem_chunk = new int[ height * width * size ];
      
//       if( verbose && !roc_mode ) { 
		
// 		printf("\n Total number of Images read (training data): \n");
// 		printf(" Class %d has %d images.\n", i, size ); 
//       }
	  
//       for(int j=0;j<size;j++){
		
// 		// allocate in one chunk 
		
// 		integralImage = new int[ height * width ]; 
// 		//	integralImage = ( mem_chunk + j * height * width ); 
		
// 		for(int h=0;h<height;h++)
// 		  for(int w=0;w<width;w++) {
			
// 			in>>var;
// 			integralImage[ h * width + w ] = var; 
			
// 		  } 
		
// 		//  printf("Image %d read.\n", j); 
		
// 		config.integralImages[i].iiVector.push_back( integralImage );
//       }
	  
	  
//       in.close(); 
//     }
	
//     integralImage = 0; 
    
	
//     //writeOutIntegralImages(integralImages,NCLASSES, height, width, cout);
//     //    in.close();
    
	
//     //************ Reading from File : End Section ************ 
    
//     config.setTestIntegralImagesLoc( TESTDT );
    
//   } // reading offline 
  
  
  
//   /*********** TEST MODE: Unit testing and to check memory leaks **********/   
//   if( test_mode )
//     _tTree( config ); 
  
  
  
//   /************* SLIDING WINDOW/ PERFORMANCE ************/ 
  
//   int* category = new int[config.getNclasses()]; // The category array ( deals with faces)
//   category[0] = 0; category[1] = 1;
  
//   int indexOfFaceCategory = 0;
  
//   //  double scales[] = {1.0, 2.0, 0.5, 3.0, 0.3, 2.5, 0.25, 0.1, 0.6, 0.8, 1.5};  // Scales used for sliding window detection scheme
  
//   double scales[] = { 1.0, 2.0, 0.5, 3.0, 0.35, 4.0, 0.25, 0.2, 5.0, 0.15, 0.75, 2.5, 3.25, 5.4, 4.7, 1.3};
  
// //  double scales[] = { 1.0, 2.0, 0.5, 3.0, 0.25}; 
  
//   //double scales[] = { 1.0, 2.0, 0.5, 3.0, 3.5, 4.0, 0.7, 0.4 };
  
//   //double scales[] = {1.0, 0.25 , 0.5, 0.8 , 0.3, 2.0 , 0.6, 0.4 , 0.1};
  
//   //  double scales[] = { 1.0, 1.0, 1.0, 1.0, 1.0 }; // Check if the program gets killed because of diff image size
  
  
//   //  double scales[] = {1.0, 0.9 , 0.8, 0.7 , 0.85 , 0.6 , 0.5, 0.4 , 0.45};
  
//   Forest::Tree* aTree = 0;
  
//   int numberOfTreesGrown = 0 ;
  
  
//   if( run_classification ) { 
// 	// if we run classificaiton, the program exits immediately afterwards
	
// 	bool make_forest = true; 
// 	using Forest::Tree; 
// 	//Tree *bTree = 0, *cTree = 0, *dTree = 0, *eTree = 0, *fTree = 0;
// 	int _forest = _FOREST; 
// 	Tree *remaining_trees[ _forest -1 ]; 

	
// 	string nameLocOfImage;
// 	int globalNodeIndex = 0; 
// 	int* tempII; 
//     aTree = new Tree( config, globalNodeIndex, numberOfTreesGrown++ ); 
    
//     if( make_forest ) 
//     {
    
//     // this is not memory safe :( 
//   /*
// 	  numberOfTreesGrown = 0;
// 	  globalNodeIndex = 0; 
// 	  bTree = new Tree( config, globalNodeIndex, numberOfTreesGrown++ ); 
	  
// 	  numberOfTreesGrown = 0;
// 	  globalNodeIndex = 0;
// 	  cTree = new Tree( config, globalNodeIndex, numberOfTreesGrown++ ); 
	  
// 	  numberOfTreesGrown = 0;
// 	  globalNodeIndex = 0;
// 	  dTree = new Tree( config, globalNodeIndex, numberOfTreesGrown++ ); 
	  
	  
// 	  numberOfTreesGrown = 0;
// 	  globalNodeIndex = 0;
// 	  eTree = new Tree( config, globalNodeIndex, numberOfTreesGrown++ ); 
	  
	  
// 	  numberOfTreesGrown = 0;
// 	  globalNodeIndex = 0;
// 	  fTree = new Tree( config, globalNodeIndex, numberOfTreesGrown++ ); 
// 	 */
	  
// 	  for( int j = 0; j < _forest -1; j++ ) 
// 	  {
// 		numberOfTreesGrown = 0; 
// 		globalNodeIndex = 0;
// 		remaining_trees[j] = new Tree( config, globalNodeIndex, numberOfTreesGrown++ ); 
// 	  }
	  
// 	}
	
    
// 	if( verbose )
// 	{
// 	  //	  printf("\n Tree%d has %d faces and %d non-faces.\n", numberOfTreesGrown, aTree->theIntegralImages[0].size(), aTree->theIntegralImages[1].size() ); 
// 	}
	
// 	if( testImageSupplied ) 
// 	  nameLocOfImage = testImagePath; 
// 	else
// 	  nameLocOfImage = TEST_IMAGE_SLIDING_WINDOW; 
	
// 	IplImage* imageToClassify = UserDef::getImageFromLocation( nameLocOfImage ); 
// 	int* imageToClassifyII = new int[ imageToClassify->height * imageToClassify->width ]; 
// 	calculateIntegralImage( imageToClassify, imageToClassifyII );
	
// 	int recognition_histogram[ _forest ]; 
	
	
// 	using Forest::runClassification; 
//     recognition_histogram[0] = runClassification( aTree, config, imageToClassifyII ); 
// 	printf("The classification number is %d\n", recognition_histogram[0] ) ;
	
// 	if( make_forest ) {
// /*	  
// 	  recognition_histogram[1] = runClassification( bTree, config, imageToClassifyII ); 
// 	  printf("The classification number is %d\n", recognition_histogram[1] ); 
	  
// 	  recognition_histogram[2] = runClassification( cTree, config, imageToClassifyII );
// 	  printf("The classification number is %d\n", recognition_histogram[2] ); 
	  
// 	  recognition_histogram[3] = runClassification( dTree, config, imageToClassifyII );
// 	  printf("The classification number is %d\n", recognition_histogram[3] ); 
	  
// 	  recognition_histogram[4] = runClassification( eTree, config, imageToClassifyII );
// 	  printf("The classification number is %d\n", recognition_histogram[4] ); 
	  
// 	  recognition_histogram[5] = runClassification( fTree, config, imageToClassifyII );
// 	  printf("The classification number is %d\n", recognition_histogram[5] ); 
// 	*/
	  
// 	  for( int j =1; j< _forest; j++ )
// 	  {
// 		recognition_histogram[j] = runClassification( remaining_trees[j-1], config, imageToClassifyII ); 
// 		printf("the classifiation number is %d\n", recognition_histogram[j] ); 
// 	  }
	  
	  
// 	  int votes[ nclasses ];
	  
// 	  for( int j = 0; j < nclasses; j++ ) 
// 		votes[j] = 0; 
	  
// 	  for( int j = 0; j < _forest; j++ )
// 		votes[ recognition_histogram[j] ]++; 
	  
// 	  int max_identity = -1; 
// 	  int index_max_identity = -1; 
	  
// 	  for( int j = 0; j < nclasses; j++ ) 
// 		if( votes[j] > max_identity ) 
// 		{
// 		  max_identity = votes[j]; 
// 		  index_max_identity = j; 
// 		}
	  
// 	  printf("the classification final result is %d\n", index_max_identity ); 
	  
// 	}
	
	
// 	 for( int i = 0; i < config.getNclasses(); i++ ) 
//     	for( int j = 0; j < config.integralImages[i].size(); j++ ) { 
	
// 	  tempII = config.integralImages[i].iiVector.at( j ); 
// 	  delete[] tempII; 
// 	}
	
// 	delete( aTree ); 
	
// 	if( make_forest )
// 	{
// 	  for( int j =0; j < _forest -1; j++ ) 
// 		delete( remaining_trees[j] );
// 	}
	
// 	exit(0); 
	
//   }    
  
  
  
//   if( performance ) { 
    
//     runPerformance( config, 
// 				   infoname, 
// 				   category, 
// 				   indexOfFaceCategory, 
// 				   saveDetected, 
// 				   dt,
// 				   forestSize,
// 				   maxPosDiff,
// 				   maxSizeDiff, 
// 				   scales, 
// 				   number_of_scale_ranges, 
// 				   roc_mode ); 
    
    
    
//   } // if(performance)..   
  
  
//   if( !performance && !test_mode ) { // default run with detections drawn: Ready to use
	
    
// 	//	THE TREES SECTION - UNCOMMMENT IF U WANT TO BUILD A FOREST
// 	// while( numberOfTreesGrown < forestSize ) { 
	
// 	// The handling of the performance of a forest will be handleded individually 
// 	// depenending on whether its (a) Classification (b) Sliding window detection 
	
// 	int globalNodeIndex = 0;
// 	config.forestWindows.clear();
	
	
// 	if( aTree != 0 ) 
// 	  dealloc_mem( aTree ); 
	
	
	
// 	//       UserDef::VectorOfIntegralImages vectorOfIntegralImages;
	
// 	//       for( int i = 0 ; i < NCLASSES ; i++ ) 
// 	// 	for( int j = 0 ; j < config.integralImages[i].iiVector.size() ; j++ ) 
// 	// 	  vectorOfIntegralImages[i].iiVector.push_back( &( config.integralImages[i].iiVector.at(j) ) ); 
	
	
// 	aTree = new Forest::Tree( config, 
// 							 globalNodeIndex, 
// 							 numberOfTreesGrown++ ); // 1. First grow the tree
	
	
	
// 	if( verbose ) {
	  
// 	  printf("\n Tree%d has %d faces and %d non-faces.\n", numberOfTreesGrown, 
// 			 aTree->theIntegralImages[0].size(), aTree->theIntegralImages[1].size() ); 
	  
// 	  /*	printf(" The following details pertain to Tree#%d :\n", 
// 	   numberOfTreesGrown );
	   
// 	   printf(" The Treelevel: %d\t Level_: %d\t integralImageSize1: %d\t integral2: %d\t\n", 
// 	   aTree->getTreeLevel(), aTree->level_, aTree->theIntegralImages[0].iiVector.size(), 
// 	   aTree->theIntegralImages[1].iiVector.size() ); 
	   
// 	   */    }
	
	
// 	/* if( identityfile != 0 )
// 	 {
// 	 */
	
// 	if( identityfileSupplied ) 
// 	{
	  
// 	  IplImage* propagateImage = UserDef::getImageFromLocation( identityfile );
// 	  int* propagateImageII = new int[ propagateImage->height * propagateImage->width ];
	  
// 	  calculateIntegralImage( propagateImage, propagateImageII );
	  
// 	  Forest::propagateTree( aTree, config, propagateImageII, propagateImage->height, 
// 							propagateImage->width ); 
	  
	  
// 	  exit(1);
// 	}
	
// 	// }
	
// 	start_detection = std::clock();
// 	for( int nos_scales = 0 ; nos_scales < number_of_scale_ranges ; nos_scales++ ) 
// 	{ 
	  
	  
// 	  UserDef::WindowClass* wc = 0;
// 	  for( unsigned int i = 0; i < config.windows.size() ; i++ )
// 	  { 
// 		wc = config.windows.at( i );
// 		delete wc; 
// 	  }
	  
// 	  config.windows.clear(); 
	  
// 	  string nameLocOfImage;
		
// 	  if( testImageSupplied ) 
// 	    nameLocOfImage = testImagePath; 
		
// 	  else
// 	    nameLocOfImage = TEST_IMAGE_SLIDING_WINDOW;

	  
// 	  if( RUN_SLIDING_WINDOW ) {   // ********** sliding_window ******** 
		
	
		
		
// 		double scaleFactor = scales[ nos_scales ];     
		
// 		if( vote==1 ) {     // Running through differentlty scaled images 
		  
// 		  IplImage* imageToResize = UserDef::getImageFromLocation(nameLocOfImage);
		  
// 		  /* 
// 		   * config.getDetectionThreshold might be deprecated. I am using the mergeDetections in place. 
// 		   */ config.setDetectionThreshold((int)( scaleFactor * dt ) );
		  
// 		  // images unable to deal with sizes greater than 1400-1440 
// 		  if( (int)( scaleFactor * imageToResize->height ) < 1300 && 
// 			 (int)( scaleFactor * imageToResize->width ) < 1300 ) { 
			
// 			bool security_check = false; 
			
// 			if( security_check ) { 
			  
// 			  if(config.getDetectionThreshold()==0 || config.getDetectionThreshold()<0)
// 				config.setDetectionThreshold(1);
			  
			  
// 			  if( 
// 				 config.imageInfoInstance.getHeight()  >
// 				 (int)( scaleFactor * config.testImageInfoInstance.getHeight() ) ||
// 				 config.imageInfoInstance.getWidth()  >
// 				 (int)( scaleFactor * config.testImageInfoInstance.getWidth() ) 
// 				 )  
// 				scaleFactor = 1.0;  // Is scale too magnified? Then, do not resize test Image
			  
// 			  if(
// 				 (int)( scaleFactor * config.testImageInfoInstance.getHeight() ) == 0  
// 				 || 
// 				 (int)( scaleFactor * config.testImageInfoInstance.getWidth() ) == 0 )
// 				scaleFactor = 1.0; // Is scale too minute? Then, do not resize test image
			  
			  
// 			}
			
// 			CvSize resizeImageSize = cvSize( (int)(scaleFactor*imageToResize->width), 
// 											(int) (scaleFactor*imageToResize->height) );
			
// 			IplImage* slidingWindowTestIplImage = cvCreateImage( resizeImageSize, 
// 																imageToResize->depth, 
// 																imageToResize->nChannels );
			
// 			cvResize( imageToResize, 
// 					 slidingWindowTestIplImage, 
// 					 CV_INTER_LINEAR );
// 			cvReleaseImage( &imageToResize );
			
// 			int* integralImageOfSlidingWindow = new int[( slidingWindowTestIplImage->height) * 
// 														(slidingWindowTestIplImage->width)];
			
// 			calculateIntegralImage(slidingWindowTestIplImage, 
// 								   integralImageOfSlidingWindow);
			
// 			config.testImageInfoInstance.setHeight(slidingWindowTestIplImage->height); //set new height
// 			config.testImageInfoInstance.setWidth(slidingWindowTestIplImage->width); //set new widht
			
// 			aTree->slidingWindow( aTree, 
// 								 integralImageOfSlidingWindow, 
// 								 config, 
// 								 category, 
// 								 indexOfFaceCategory, 
// 								 aTree->getForestIndex(), 
// 								 scaleFactor );
			
			
// 			config.globalDetectionsFile<<"\n---------End of Tree: Before Merges------------\n";
// 			Auxillary::printConfigWindows( config, config.windows ); 
			
			
// 			config.globalDetectionsFile<<"\n---------End of Tree: After Merges------------\n";      
			
// 			mergeDetections( config.windows, config , scaleFactor ); 
			
// 			Auxillary::printConfigWindows( config, config.windows ); 
			
			
// 			//	  if( forestSize > 1 ) {  // integrate the results of each tree
			

			
// 			UserDef::filterNoise6( config.windows, config );
			 
			
// 			//filterNoise( config.windows, config ); 
// 			dummy_combineVotes(config, 
// 							   NEIGHBOR_RANGE, 
// 							   scaleFactor ); 
// 			// I am going to try to merge the windows from a scale
// 			//			filterNoise( config.windows, config ); 
			
// 			//	  } 
			
// 			char nameAdd[ 10 ] ;
// 			sprintf( nameAdd , "%d#%d-" , numberOfTreesGrown , nos_scales );  // Unique name
			
		       

// 			if( saveDetected )
// 			  UserDef::drawDetections( slidingWindowTestIplImage, 
// 									  config.windows, 
// 									  nameLocOfImage, 
// 									  nameAdd, 
// 									  saveDetected );
			
// 			cvReleaseImage( &slidingWindowTestIplImage );
// 			delete[] integralImageOfSlidingWindow; 
			
// 		  } // height, width < 1400
		  
// 		} // if(vote==1).. 
		
		
// 		if(vote==2){
		  
// 		  ERR_MODULE("int main: vote==2");
// 		  exit( 1 ); 
		  
		  
// 		  // 	    string nameLocOfImage;
		  
// 		  // 	    if(testImageSupplied) 
// 		  // 	      nameLocOfImage = testImagePath; 
// 		  // 	    else
// 		  // 	      nameLocOfImage = TEST_IMAGE_SLIDING_WINDOW;
		  
// 		  // 	    double scaleFactor = scales[numberOfTreesGrown-1]; 
		  
// 		  // 	    // Since in the case of vote==2, I am increasing expecting bigger and bigger 
// 		  // 	    // rectangles, I consider the division inverse of the scale Factor
// 		  // 	    scaleFactor = 1.0F/scaleFactor;
		  
		  
// 		  // 	    IplImage* slidingWindowTestIplImage = UserDef::getImageFromLocation(nameLocOfImage);
		  
		  
// 		  // 	    config.setDetectionThreshold( (int)( scaleFactor * dt ) );
		  
// 		  // 	    if( config.getDetectionThreshold() == 0 || 
// 		  // 		config.getDetectionThreshold() < 0 )
		  
// 		  // 	      config.setDetectionThreshold(1);
		  
		  
// 		  // 	    int* integralImageOfSlidingWindow = new int[(slidingWindowTestIplImage->height)
// 		  // 							*(slidingWindowTestIplImage->width)];
		  
// 		  // 	    calculateIntegralImage(slidingWindowTestIplImage, integralImageOfSlidingWindow);
		  
		  
// 		  // 	    config.testImageInfoInstance.setHeight(slidingWindowTestIplImage->height);
// 		  // 	    config.testImageInfoInstance.setWidth(slidingWindowTestIplImage->width);
		  
// 		  // 	    if(
// 		  // 	       (int)( scaleFactor * config.imageInfoInstance.getHeight()) > 
// 		  // 	       config.testImageInfoInstance.getHeight() 
// 		  // 	       || 
// 		  // 	       (int)(scaleFactor * config.imageInfoInstance.getWidth()) > 
// 		  // 	       config.testImageInfoInstance.getWidth()
// 		  // 	       ) 
// 		  // 	    scaleFactor = 1.0;  // It could the scale is too high
		  
// 		  // 	  if(
// 		  // 	     (int)( scaleFactor * config.imageInfoInstance.getHeight()) == 0 
// 		  // 	     || (int)( scaleFactor * config.imageInfoInstance.getWidth()) == 0
// 		  // 	     ) 
// 		  // 	    scaleFactor = 1.0; //the scale is too low
		  
// 		  // 	  aTree->slidingWindowWithScaling(aTree, 
// 		  // 					  integralImageOfSlidingWindow, 
// 		  // 					  config, 
// 		  // 					  category, 
// 		  // 					  indexOfFaceCategory, 
// 		  // 					  scaleFactor);
		  
// 		  // 	  //!!!!!!!!!!!!!NOT CALLING MERGE WINDOWS
// 		  // 	  /*Before calling drawDetections, I wil merge 
// 		  // 	    subwindows to reduce the number of false positives - 
// 		  // 	    in line with what has been done in \cite{violaJones2001} */
// 		  // 	  //       UserDef::mergeDetectionSubwindow(config.windows, 
// 		  // 	  // 				       config.imageInfoInstance.getHeight(), 
// 		  // 	  // 				       config.imageInfoInstance.getWidth(), 
// 		  // 	  // 				       config);
		  
		  
		  
		  
// 		  // 	  Auxillary::printConfigWindows( config, config.windows ); 
// 		  // 	  config.globalDetectionsFile<<"\n---------End of Tree: Before Merges------------\n";
		  
// 		  // 	  mergeDetections( config.windows, config , scaleFactor ); 
// 		  // 	  Auxillary::printConfigWindows( config, config.windows ); 
// 		  // 	  config.globalDetectionsFile<<"\n---------End of Tree: After Merges------------\n";
		  
// 		  // 	  if( forestSize > 1 ) {  // integrate the results of each tree
		  
// 		  // 	    // Here, the combine votes actually rescales the windows. Usually, in case of vote==1 , 
// 		  // 	    // the windows are always of the size 19X19 and the images are scaled as well. 
// 		  // 	    // So when we send the detections to the combine module, we scale them appropriately.
// 		  // 	    // On the other hand, in the case of vote==2, we simply neither need to scale the window sizes
// 		  // 	    // nor to scale window sizes. So, I call combineVotes with 1/scaleFactor
		  
// 		  // 	    // !! This does not work because although the window coordinates end up being correctly transformed
// 		  // 	    // (i.e. they shoudl remain the same), the actual coordinates get transformed. 
		  
// 		  // 	    combineVotes_2( config, 
// 		  // 			    NEIGHBOR_RANGE ); 
// 		  // 	  } 
		  
		  
		  
// 		  // 	  char nameAdd[10];
// 		  // 	  sprintf(nameAdd,"%d-", numberOfTreesGrown);
		  
// 		  // 	  if( saveDetected )
// 		  // 	    UserDef::drawDetections(slidingWindowTestIplImage, 
// 		  // 				    config.windows, 
// 		  // 				    nameLocOfImage, 
// 		  // 				    nameAdd, 
// 		  // 				    saveDetected);
		  
		  
// 		  // 	  cvReleaseImage(&slidingWindowTestIplImage);
		  
// 		} // if(vote==2)...
		
// 	  } //if(RUN_SLIDIN..
	  
	  
	  
// 	} // for( nos_scales .. 
	
	
// 	// IF SLIDINGWINDOW was called, draw final set of detections
// 	if(RUN_SLIDING_WINDOW) { 
	  
	  
// 	  config.globalDetectionsFile<<"\n---GLOBAL DETECTIONS-BEFORE TRIM--\n"; 
// 	  Auxillary::printConfigWindows( config, config.forestWindows );   
	  
// 	  // am gonna run filternoise5 for the final

// //	  filterNoise5( config.forestWindows, config, 1.0, 1 ); 
// //	   filterNoise6( config.forestWindows, config ); 

// //probably need to run filterNoise3 to get the se of windows removed and then run
// //filternoise5 module 0  The filternoise3 kind of oput everything inside everytghin else within the range of area_windowo_merge and the n filernoise5 module 0 starts to tak eaverages. 

// //    filterNoise3( config.forestWindows, config, 1.0 );
//  //   filterNoise3( config.forestWindows, config, 1.0 ); 

// //    filterNoise5( config.forestWindows, config, 1.0, 1 ); //first put windows together
//  //   filterNoise5( config.forestWindows, config, 1.0, 0 ); // then emrge by averaging
 
//     filterNoise5( config.forestWindows, config, 1.0, 1 ); 
//     filterNoise5( config.forestWindows, config, 1.0, 1 ); 

	  
// 	  printf("the number of windows are %zu", config.forestWindows.size() );
	   
// 	  for(unsigned int q = 0; q < config.forestWindows.size(); q++ ) 
// 	    {
	      
// 	      printf("%d %d %d %d %f\n", config.forestWindows.at(q)->getX(), 
// 		     config.forestWindows.at(q)->getY(), 
// 		     config.forestWindows.at(q)->getHeight(), 
// 		     config.forestWindows.at(q)->getWidth(), 
// 		     config.forestWindows.at(q)->getFaceProbabilityResult() );
	       
// 	    }
	  


	  
// 	  IplImage* slidingWindowTestIplImage = UserDef::getImageFromLocation(testImagePath);
// 	  char nameAdd[ 20 ];
// 	  sprintf( nameAdd, "final_%d-", numberOfTreesGrown ); 
	  
// 	  bool step1 = false;
// 	  //bool step2 = false; 
	  
// 	  //    comparePerformance();
	  
// 	  end_detection = std::clock(); 
// 	  config.globalDetectionsFile<<"\n---GLOBAL DETECTIONS-AFTER-TRIM--\n"; 
// 	  if( saveDetected )
// 	    UserDef::drawDetections(slidingWindowTestIplImage, 
// 				    config.forestWindows, testImagePath, nameAdd, saveDetected);
	  
// 	  Auxillary::printConfigWindows(config, config.forestWindows); 
// 	  cvReleaseImage( &slidingWindowTestIplImage ); 

// 	  slidingWindowTestIplImage = UserDef::getImageFromLocation( testImagePath ); 
	  
	  
// 	  //now we crop the images
// 	  vector< IplImage* > faces; 
// 	  for( unsigned int i = 0; i < config.forestWindows.size(); i++ ) 
// 	  {
// 		IplImage* face; 
		
// 		if( ALLOW_CROP_BOUNDARY_HEURISTIC && config.forestWindows.at(i)->getX() > 5 && 
// 		   config.forestWindows.at(i)->getY() > 5 )
// 		{
// 		  face = cvCreateImage( cvSize( config.forestWindows.at(i)->getWidth() + 10,
// 						config.forestWindows.at(i)->getHeight() + 10 ),
// 					slidingWindowTestIplImage->depth, 
// 					slidingWindowTestIplImage->nChannels ); 
		  
// 		  cvSetImageROI( slidingWindowTestIplImage, 
// 						cvRect( config.forestWindows.at(i)->getX() - 5, 
// 							   config.forestWindows.at(i)->getY() - 5, 
// 							   config.forestWindows.at(i)->getWidth() + 10, 
// 							   config.forestWindows.at(i)->getHeight() + 10 ) ); 
		  
// 		}
		
// 		else 
// 		{
		  
// 		  face = cvCreateImage( cvSize( config.forestWindows.at(i)->getWidth(), 
// 						config.forestWindows.at(i)->getHeight() ), 
// 					slidingWindowTestIplImage->depth, 
// 					slidingWindowTestIplImage->nChannels ); 
		  
// 		  cvSetImageROI( slidingWindowTestIplImage, 
// 						cvRect( config.forestWindows.at(i)->getX(),
// 							   config.forestWindows.at(i)->getY(),
// 							   config.forestWindows.at(i)->getWidth(),
// 							   config.forestWindows.at(i)->getHeight() ) );
		  
// 		}
// 		cvCopy( slidingWindowTestIplImage, face, 0 ); 
		
// 		char number[10];
// 		sprintf( number, "%d", i);
		
// 		char face_storage[PATH_MAX + 50];
// 		strcpy( face_storage, testImagePath ); 
// 		strcat( face_storage, "-crop-" );
// 		strcat( face_storage, number );
// 		strcat( face_storage, ".png" ); 
		
		
// 		// printf("it will be rescaled to %d %d \n", config.imageInfoInstance.getWidth(), 
// 		//	 config.imageInfoInstance.getHeight() ); 
		
// 		CvSize scaledCropImageSize = cvSize( config.imageInfoInstance.getWidth(), 
// 											config.imageInfoInstance.getHeight() 
// 											); 
// 		IplImage *scaledFace = cvCreateImage( scaledCropImageSize, 
// 											 face->depth,
// 											 face->nChannels );
		
// 		cvResize( face, 
// 				 scaledFace,
// 				 CV_INTER_LINEAR ); 
		
// 		cvvSaveImage( face_storage, scaledFace );
		
// 		cvReleaseImage( &scaledFace ); 
		
// 		cvResetImageROI( slidingWindowTestIplImage ); 
		
		
// 		if( step1 ) 
// 		{
		  
// 		  faces.push_back ( face ); 
		  
// 		  // now, train ght enew tree
// 		  // run face dwon the new tree
// 		  //and return histogram answer
		  
// 		}
		
		
// 		cvReleaseImage( &face );
		
		
// 	  }
	  
	  
// // 	  if( saveDetected )
// // 		UserDef::drawDetections(slidingWindowTestIplImage, 
// // 								config.forestWindows, testImagePath, nameAdd, saveDetected);
	  
	  
// 	  cvReleaseImage(&slidingWindowTestIplImage);
	  
// 	}	 // if(RUN_SLID.. 
	
	
	
// 	//  }  // while(numberOfTrees..)
	
//     // IF SLIDINGWINDOW was called, draw final set of detections
//     //   if(RUN_SLIDING_WINDOW) { 
	
	
// 	// 	config.globalDetectionsFile<<"\n---GLOBAL DETECTIONS-BEFORE TRIM--\n"; 
// 	// 	Auxillary::printConfigWindows( config, config.forestWindows );   
	
// 	// 	using UserDef::filterNoise1; 
// 	// 	using UserDef::filterNoise2; 
// 	// 	filterNoise2( config, config.forestWindows);
	
// 	// 	IplImage* slidingWindowTestIplImage = UserDef::getImageFromLocation(testImagePath);
// 	// 	char *nameAdd = "final-";
	
// 	// 	//    comparePerformance();
	
// 	// 	config.globalDetectionsFile<<"\n---GLOBAL DETECTIONS-AFTER-TRIM--\n"; 
// 	// 	Auxillary::printConfigWindows(config, config.forestWindows); 
	
	
	
// 	// 	if( saveDetected )
// 	// 	  UserDef::drawDetections(slidingWindowTestIplImage, 
// 	// 				  config.forestWindows, testImagePath, nameAdd, saveDetected);
	
// 	// 	cvReleaseImage(&slidingWindowTestIplImage);
// 	//       }	 // if(RUN_SLID.. 
    
	
//     if(LEARN_OFFLINE){
	  
//       ERR_NWORKING; 
	  
//       /* ---------- may not be working ---------- */
	  
//       /*
// 	   std::ofstream treeout, testout; 
	   
// 	   treeout.open(TRAINTREE);
// 	   testout.open(TRAINTESTS);
	   
// 	   writeToFile(treeout, testout, *aTree);
	   
// 	   treeout.close();
// 	   testout.close();
	   
// 	   Forest::Tree* bTree =0;//= new Forest::Tree(0,0,0,0,0);
// 	   std::ifstream treein, testin;
	   
// 	   treein.open(TRAINTREE);
// 	   testin.open(TRAINTESTS);
	   
// 	   readFromFile(treein, testin, &bTree);
// 	   treein.close();
// 	   testin.close();
	   
// 	   std::ofstream treeout1, testout1;
	   
// 	   treeout1.open("../info/traintree1.dat");
// 	   testout1.open("../info/traintests.dat");
	   
// 	   writeToFile(treeout1, testout1, *bTree);
// 	   treeout1.close();
// 	   testout1.close();
// 	   */
//     }  
	
//   } // if(!performance)
  
  
  
  
//   /* File write 
//    FILE* fp;
//    fp = fopen( "../info/tree.dat", "w" ); 
   
   
//    Forest::writeToFile( aTree, fp );
   
//    fclose( fp ); 
//    */
  
  
//   /* dealloc the mem */ 
//   //  if( config.getRelearnStatusValue() ) { 
  
//   for( int i = 0; i < config.getNclasses(); i++ ) 
// 	for( int j = 0; j < config.integralImages[i].size(); j++ ) { 
	  
	  
// 	  tempII = config.integralImages[i].iiVector.at( j ); 
// 	  delete[] tempII; 
// 	}
  
//   // }
  
  
//   //   else 
//   //     delete[] mem_chunk; 
  
//   //  }
  
//   tempII = 0; 
//   mem_chunk = 0; 
  
//   delete[] category; 
  
//   delete aTree; 
  
//   final(config);
  
//   finish = std::clock();
  
//   if( !roc_mode) 
// 	cout<<endl<<"The time taken for complete execution in seconds:"<<(double)(finish - start)/( 1.0* CLOCKS_PER_SEC)<<endl;
  
//   if( !roc_mode )
// 	cout<<endl<<"The time taken for detection alone in seconds:"<<(double)(end_detection - start_detection)/(1.0* CLOCKS_PER_SEC)<<endl;
  
  return 0;
}

/// @endcond

