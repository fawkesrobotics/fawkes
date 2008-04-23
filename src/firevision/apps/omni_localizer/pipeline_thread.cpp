/*
    Copyright (c) 2008 Volker Krause <volker.krause@rwth-aachen.de>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*/

#include <apps/omni_localizer/pipeline_thread.h>
#include <apps/omni_localizer/lineclassifier.h>
#include <apps/omni_localizer/mcl.h>
#include <apps/omni_localizer/debug.h>

#include <fvutils/color/conversions.h>
#include <fvutils/ipc/shm_image.h>
#include <fvutils/readers/pnm.h>
#include <fvutils/writers/jpeg.h>
#include <fvutils/draw/drawer.h>

#include <models/scanlines/star.h>
#include <models/color/lookuptable.h>
#include <models/mirror/bulb.h>
#include <cams/camera.h>

#include <interfaces/motor.h>
#include <interfaces/object.h>

#include <string>
#include <iostream>
#include <cmath>

using namespace std;

/** @class FvOmniLocalizerPipelineThread <apps/omni_localizer/pipeline_thread.h>
 * Omnivision self-localization.
 *
 * @author Volker Krause
 */

/** Constructor. */
FvOmniLocalizerPipelineThread::FvOmniLocalizerPipelineThread() :
    Thread( "FvOmniLocalizerThread", Thread::OPMODE_WAITFORWAKEUP ),
    VisionAspect( VisionAspect::CYCLIC ),
    mCamera( 0 ),
    mScanlineModel( 0 ),
    mColorModel( 0 ),
    mClassifier( 0 ),
    mShmBuffer( 0 ),
    mBuffer( 0 ),
    mMirror( 0 ),
    mMCL( 0 ),
    mMotorInterface( 0 ),
    mPositionInterface( 0 ),
    mColorspaceFrom( CS_UNKNOWN ),
    mColorspaceTo( YUV422_PLANAR ),
    mUseBallPosition( false )
{
}

/** Destructor. */
FvOmniLocalizerPipelineThread::~FvOmniLocalizerPipelineThread()
{
  delete mScanlineModel;
  delete mColorModel;
  delete mClassifier;
  delete mMirror;
  delete mMCL;
}

void FvOmniLocalizerPipelineThread::init()
{
  logger->log_debug( name(), "FvOmniLocalizerPipelineThread::init()" );

  // ### default config values, where are they put usually?
  config->set_default_int( "/firevision/omni/localizer/segments", 180 );
  config->set_default_int( "/firevision/omni/localizer/radius_increase", 2 );
  config->set_default_int( "/firevision/omni/localizer/max_radius", 575 );
  config->set_default_bool( "/firevision/omni/localizer/use_ball_position", false );

  string colormapFile, mirrorFile, maskFile;
  int num_segments, radius_increase, maxRadius;
  try
  {
    colormapFile = config->get_string( "/firevision/omni/localizer/colormap" );
    mirrorFile =config->get_string("/firevision/omni/mirror");
    num_segments = config->get_int( "/firevision/omni/localizer/segments" );
    radius_increase = config->get_int( "/firevision/omni/localizer/radius_increase" );
    maxRadius = config->get_int( "/firevision/omni/localizer/max_radius" );
    maskFile = config->get_string( "/firevision/omni/mask" );
    mUseBallPosition = config->get_bool( "/firevision/omni/localizer/use_ball_position" );
  }
  catch ( Exception &e )
  {
    e.append( "FvOmniLocalizerPipelineThread::init() failed since required config entries are missing" );
    throw;
  }

#ifdef DEBUG_UNWRAP
  mLowerRange = config->get_float( "/firevision/omni/localizer/lower_range" );
  mUpperRange = config->get_float( "/firevision/omni/localizer/upper_range" );
#endif

  try
  {
    mCamera = vision_master->register_for_camera( config->get_string("/firevision/omni/camera").c_str(), this );
  }
  catch ( Exception& e )
  {
    e.append( "FvOmniLocalizerPipelineThread::init() failed since no camera is specified" );
    throw;
  }

  mImageWidth = mCamera->pixel_width();
  mImageHeight = mCamera->pixel_height();
  mColorspaceFrom = mCamera->colorspace();

  string colormapPath = string( CONFDIR ) + "/firevision/" + colormapFile;
  mColorModel = new ColorModelLookupTable( colormapPath.c_str(),
                                           "omni-localizer-colormap", true /* destroy on delete */ );

  mShmBuffer = new SharedMemoryImageBuffer( "omni-localizer-processed", mColorspaceTo, mImageWidth, mImageHeight );
  mBuffer = mShmBuffer->buffer();

  string maskPath = string( CONFDIR ) + "/firevision/" + maskFile;
  PNMReader reader( maskPath.c_str() );
  mMask = malloc_buffer( YUV422_PLANAR, reader.pixel_width(), reader.pixel_height() );
  reader.set_buffer( mMask );
  reader.read();

  string mirrorPath = string( CONFDIR ) + "/firevision/" + mirrorFile;
  mMirror = new Bulb( mirrorPath.c_str(), "omni-localizer-mirror", true /* destroy on delete */ );

  mScanlineModel = new ScanlineStar( mImageWidth, mImageHeight,
                                     mMirror->getCenter().x,
                                     mMirror->getCenter().y,
                                     num_segments, radius_increase,
                                     mMask,
                                     0 /* dead radius */, maxRadius,
                                     5 /* margin */ );

  try {
    mClassifier = new LineClassifier( mScanlineModel, mColorModel, config );
  } catch ( Exception &e ) {
    e.append( "Creating classifier failed!" );
    throw;
  }

  try {
    mMCL = new MCL( blackboard, config );
  } catch ( Exception &e ) {
    e.append( "Creating MCL failed!" );
    throw;
  }

  try {
    mMotorInterface = blackboard->open_for_reading<MotorInterface>( "Motor" );
    mMotorInterface->msgq_enqueue( new MotorInterface::ResetOdometryMessage );
  } catch ( Exception& e ) {
    e.append( "Opening motor interface failed!" );
    throw;
  }

  try {
    mPositionInterface = blackboard->open_for_writing<ObjectPositionInterface>( "OmniLocalize" );
    mPositionInterface->set_object_type( ObjectPositionInterface::TYPE_SELF );
    mPositionInterface->set_visible( true );
    mPositionInterface->set_flags( ObjectPositionInterface::FLAG_HAS_WORLD |
                                   ObjectPositionInterface::FLAG_HAS_COVARIANCES |
                                   ObjectPositionInterface::FLAG_HAS_Z_AS_ORI );
    mPositionInterface->write();
  } catch ( Exception &e ) {
    e.append( "Opening object position interface failed!" );
    throw;
  }

  try {
    list<ObjectPositionInterface *> *lst = blackboard->open_all_of_type_for_reading<ObjectPositionInterface>("Ball");
    for ( list<ObjectPositionInterface *>::const_iterator it = lst->begin(); it != lst->end(); ++it )
      mBallInterfaces.push_back( *it );
    logger->log_debug( name(), "Found %i ball position interfaces", mBallInterfaces.size() );
    delete lst;
  } catch ( Exception &e ) {
    e.append( "Opening ball interfaces failed!" );
  }

  bbio_add_interface_create_type( "ObjectPositionInterface" );
  blackboard->register_observer( this, BlackBoard::BBIO_FLAG_CREATED );
}

void FvOmniLocalizerPipelineThread::finalize()
{
  logger->log_debug( name(), "FvOmniLocalizerPipelineThread::finalize()" );
  blackboard->unregister_observer( this );
  vision_master->unregister_thread( this );
  delete mCamera;
  mBuffer = 0;
  delete mShmBuffer;
  mShmBuffer = 0;
  free(mMask);
  mMask = 0;

  try {
    blackboard->close( mMotorInterface );
    for ( vector<ObjectPositionInterface*>::const_iterator it = mBallInterfaces.begin(); it != mBallInterfaces.end(); ++it )
      blackboard->close( *it );
  } catch (Exception& e) {
    e.append( "Failed to close interfaces!" );
    throw;
  }

  try {
    mPositionInterface->set_visible( false );
    mPositionInterface->write();
    blackboard->close( mPositionInterface );
  } catch ( Exception &e ) {
    e.append( "Failed to close object position interface!" );
    throw;
  }
}

#define DEBUG_IMAGE_SCALE 100.0

void FvOmniLocalizerPipelineThread::loop()
{
#ifdef DEBUG_MCL_LOG
  static int loopCount = 0;
  cout << "*** entering loop " << loopCount << " ***" << endl;
  mMCL->mLoopCount = loopCount;
#endif

  mCamera->capture();
  convert( mColorspaceFrom, mColorspaceTo, mCamera->buffer(), mBuffer, mImageWidth, mImageHeight );
  mCamera->dispose_buffer();

  mClassifier->set_src_buffer( mBuffer, mImageWidth, mImageHeight );
#ifdef DEBUG_MCL_LOG
  mClassifier->mLoopCount = loopCount;
#endif
  const map<float, vector<f_point_t> > sensorHits = mClassifier->classify2();

  // predict new position based on odometry
  field_pos_t odometry;
  mMotorInterface->read();
  odometry.x = mMotorInterface->odometry_position_x();
  odometry.y = mMotorInterface->odometry_position_y();
  odometry.ori = mMotorInterface->odometry_orientation();
  const float pathLength = mMotorInterface->odometry_path_length();
  mMotorInterface->msgq_enqueue( new MotorInterface::ResetOdometryMessage );
  std::cout << "got incremental odometry: (" << odometry.x << "," << odometry.y << ") " << odometry.ori << " pl: " << pathLength << endl;
  mMCL->predict( odometry, pathLength );

#ifdef DEBUG_UNWRAP
  unsigned char *debugBuffer;
  unsigned int debugBufferSize = colorspace_buffer_size( YUV422_PLANAR, DEBUG_IMAGE_SIZE, DEBUG_IMAGE_SIZE );
  debugBuffer = new unsigned char[debugBufferSize];
  memset( debugBuffer, 0, debugBufferSize );
  Drawer debugDrawer;
  debugDrawer.setBuffer( debugBuffer, DEBUG_IMAGE_SIZE, DEBUG_IMAGE_SIZE );

  // dump unwrapped image
  for ( unsigned int x = 0; x < mImageWidth; ++x ) {
    for ( unsigned int y = 0; y < mImageHeight; ++y ) {
      polar_coord_t unwrap = mMirror->getWorldPointRelative( x, y );
      unwrap.phi += MIRROR_ROTATION_OFFSET;
      if ( unwrap.r > 0 ) {
        unsigned int xuw, yuw;
        xuw = (unsigned int)( DEBUG_IMAGE_SIZE/2.0 + ( unwrap.r * cos( unwrap.phi ) * DEBUG_IMAGE_SCALE ) );
        yuw = (unsigned int)( DEBUG_IMAGE_SIZE/2.0 + ( unwrap.r * sin( unwrap.phi ) * DEBUG_IMAGE_SCALE ) );
        unsigned char yc, uc, vc;
        YUV422_PLANAR_YUV( mBuffer, mImageWidth, mImageHeight, x, y, yc, uc, vc);
        if ( xuw < DEBUG_IMAGE_SIZE && yuw < DEBUG_IMAGE_SIZE ) {
          debugDrawer.setColor( yc, uc ,vc );
          debugDrawer.drawPoint( xuw, yuw );
        }
      }
    }
  }

  // lower/upper limits
  debugDrawer.setColor( 128, 128, 255 );
  debugDrawer.drawCircle( DEBUG_IMAGE_SIZE / 2, DEBUG_IMAGE_SIZE / 2, (unsigned int)(DEBUG_IMAGE_SCALE * mLowerRange) );
  debugDrawer.drawCircle( DEBUG_IMAGE_SIZE / 2, DEBUG_IMAGE_SIZE / 2, (unsigned int)(DEBUG_IMAGE_SCALE * mUpperRange) );
  debugDrawer.setColor( 128, 0, 255 );
#endif

  // unwrap sensor readings
  map< float, vector<polar_coord_t> > convertedHits;
  int hitCount = 0;
  for ( map< float, vector<f_point_t> >::const_iterator rayIt = sensorHits.begin(); rayIt != sensorHits.end(); ++rayIt ) {
    float avgPhi = 0.0;
    vector<polar_coord_t> v;
    for ( vector<f_point_t>::const_iterator hitIt = (*rayIt).second.begin(); hitIt != (*rayIt).second.end(); ++hitIt ) {
      polar_coord_t unwrap = mMirror->getWorldPointRelative( (int)(*hitIt).x, (int)(*hitIt).y );
      if ( unwrap.r > 0 ) {
        ++hitCount;
        v.push_back( unwrap );
        avgPhi += unwrap.phi;
#ifdef DEBUG_UNWRAP
        unsigned int x, y;
        x = (unsigned int)( DEBUG_IMAGE_SIZE/2.0 + ( unwrap.r * cos( unwrap.phi ) * DEBUG_IMAGE_SCALE ) );
        y = (unsigned int)( DEBUG_IMAGE_SIZE/2.0 + ( unwrap.r * sin( unwrap.phi ) * DEBUG_IMAGE_SCALE ) );
        if ( x < DEBUG_IMAGE_SIZE && y < DEBUG_IMAGE_SIZE )
          debugDrawer.drawCircle( x, y, 3);
#endif
      }
    }

    // convert the ray angle
    // ### this assumes that the mirror doesn't affect phi over the full range of r
    // which is only nearly true (error is a few 10^-1 degree
    // (sic) see line classifier for cartesian coordinate calculation
    float rayPhi;
    if ( v.size() == 0 ) {
      const polar_coord_t unwrap = mMirror->getWorldPointRelative(
          (int)(sin( (*rayIt).first ) * 200 + mMirror->getCenter().x),
          (int)(cos( (*rayIt).first ) * 200 + mMirror->getCenter().y) );
      rayPhi = unwrap.phi;
    } else {
      rayPhi = avgPhi/v.size();
    }
    convertedHits[ rayPhi ] = v;
  }

  // get locally determined ball position(s)
  vector<f_point_t> ballHits;
  if ( mUseBallPosition ) {
    for ( vector<ObjectPositionInterface*>::const_iterator it = mBallInterfaces.begin(); it != mBallInterfaces.end(); ++it ) {
      // TODO
      (*it)->read();
      f_point_t p;
      p.x = (*it)->relative_x();
      p.y = (*it)->relative_y();
      ballHits.push_back( p );
    }
  }

  // update and resample if we have new sensor readings
  if ( hitCount == 0 ) {
    logger->log_warn( name(), "No sensor readings found." );
  } else {
    mMCL->update( convertedHits );
    if ( mUseBallPosition )
      mMCL->updateBall( ballHits );
    mMCL->resample();
  }
  mMCL->calculatePose();

#ifdef DEBUG_UNWRAP
  JpegWriter writer;
  writer.set_dimensions( DEBUG_IMAGE_SIZE, DEBUG_IMAGE_SIZE );
  writer.set_buffer( YUV422_PLANAR, debugBuffer );
  char unwrapFileName[20];
  snprintf( unwrapFileName, 20, "unwrapped-%04d.jpg", loopCount );
  writer.set_filename( unwrapFileName );
  writer.write();
  delete[] debugBuffer;
#endif

#ifdef DEBUG_MCL_STATE
  char fileName[20];
  snprintf( fileName, 20, "mclstate-%04d.jpg", loopCount );
  mMCL->dumpState( fileName );
#endif

  field_pos_t currentPos = mMCL->pose();
  field_pos_t currentVar = mMCL->variance();
  mPositionInterface->set_world_x( currentPos.x );
  mPositionInterface->set_world_y( currentPos.y );
  mPositionInterface->set_world_z( currentPos.ori );
  float *cov = new float[9];
  memset( cov, 0, 9 );
  cov[0] = currentVar.x;
  cov[4] = currentVar.y;
  cov[8] = currentVar.ori;
  mPositionInterface->set_world_xyz_covariance( cov );
  mPositionInterface->write();
  delete[] cov;

#ifdef DEBUG_MCL_LOG
  ++loopCount;
#endif
}

void FvOmniLocalizerPipelineThread::bb_interface_created(const char * type, const char * id) throw(  )
{
  cout << "interface created: " << id << endl;
  if ( strncmp( id, "Ball", strlen("Ball") ) == 0 ) {
    logger->log_debug( name(), "Found new ball position interface: %s", id );
    try {
      ObjectPositionInterface *ballIface = blackboard->open_for_reading<ObjectPositionInterface>( id );
      mBallInterfaces.push_back( ballIface );
    } catch ( Exception &e ) {
      cout << "Opening interface failed: " << e.what() << endl;
    }
  }
}
