
/***************************************************************************
 *  mcl.cpp - Monte Carlo Localization
 *
 *  Created: ???
 *  Copyright  2008  Volker Krause <volker.krause@rwth-aachen.de>
 *
 *  $Id: pipeline_thread.cpp 1049 2008-04-24 22:40:12Z beck $
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include <apps/omni_localizer/mcl.h>
#include <apps/omni_localizer/field.h>
#include <apps/omni_localizer/debug.h>
#include <apps/omni_localizer/utils.h>

#include <fvutils/color/conversions.h>
#include <fvutils/writers/jpeg.h>
#include <fvutils/draw/drawer.h>

#include <blackboard/blackboard.h>
#include <config/config.h>

#include <iostream>
#include <cstdlib>
#include <cmath>

using namespace fawkes;

// #define RESAMPLE_ADJUST(x) x
#define RESAMPLE_ADJUST(x) (powf(x, 2.0f))
// #define RESAMPLE_ADJUST(x) (0.1f + (x*0.9f))
// #define RESAMPLE_ADJUST(x) ( powf( x - 0.5f, 5.0f ) + powf( 0.5f, 5.0f ) )

#define SENSOR_COUNT_WEIGHT(x) 1
// #define SENSOR_COUNT_WEIGHT(x) ( min(1.0f, (0.02f*x + 0.8f) ) )
// #define SENSOR_COUNT_WEIGHT(x) ( min(1.0f, (0.04f*x)) )

// sing namespace std;

/** @class MCL <apps/omni_localizer/mcl.h>
 * Monte Carlo Localization.
 *
 * @author Volker Krause
 */

static bool operator<( const mcl_sample_t &left, const mcl_sample_t &right )
{
  return left.weight < right.weight;
}

static bool operator<( const mcl_sample_t &left, float right )
{
  return left.weight < right;
}

static bool operator<( float left, const mcl_sample_t &right )
{
  return left < right.weight;
}

/**
  Constructor.
  @param blackboard The blackboard.
  @param config The configuration to setup MCL.
*/
MCL::MCL( BlackBoard *blackboard, Configuration *config ) :
    mNewOdometry( false ),
    mUniformDist( mRng )
{
  // defaults
  config->set_default_string( "/firevision/omni/localizer/field", "msl2007.field" );
  config->set_default_uint( "/firevision/omni/localizer/mcl/sample_count", 200 );
  config->set_default_float( "/firevision/omni/localizer/mcl/free_sample_ratio", 0.0 );
  config->set_default_float( "/firevision/omni/localizer/mcl/idle_regeneration_threshold", 0.3 );
  config->set_default_float( "/firevision/omni/localizer/mcl/variance_limit_x", 3.0 );
  config->set_default_float( "/firevision/omni/localizer/mcl/variance_limit_y", 3.0 );
  config->set_default_float( "/firevision/omni/localizer/mcl/variance_limit_ori", M_PI/2.0 );
  config->set_default_float( "/firevision/omni/localizer/mcl/unexpected_penalty", -0.5 );
  config->set_default_float( "/firevision/omni/localizer/mcl/unexpected_penalty_min_distance", 0.4 );
  config->set_default_float( "/firevision/omni/localizer/mcl/out_of_field_threshold", 0.2 );
  config->set_default_float( "/firevision/omni/localizer/obstacle_position_weight", 0.1 );

  // read config settings
  mField = new Field( blackboard, config );

  mSampleCount = config->get_uint( "/firevision/omni/localizer/mcl/sample_count" );
  mFreeSampleRatio = config->get_float( "/firevision/omni/localizer/mcl/free_sample_ratio" );
  mIdleRegenerationThreshold = config->get_float( "/firevision/omni/localizer/mcl/idle_regeneration_threshold" );
  mVarianceLimit.x = config->get_float( "/firevision/omni/localizer/mcl/variance_limit_x" );
  mVarianceLimit.y = config->get_float( "/firevision/omni/localizer/mcl/variance_limit_y" );
  mVarianceLimit.ori = config->get_float( "/firevision/omni/localizer/mcl/variance_limit_ori" );
  mUnexpectedPenalty = config->get_float( "/firevision/omni/localizer/mcl/unexpected_penalty" );
  mUnexpectedPenaltyMinDistance = config->get_float( "/firevision/omni/localizer/mcl/unexpected_penalty_min_distance" );
  mOutOfFieldThreshold = config->get_float( "/firevision/omni/localizer/mcl/out_of_field_threshold" );
  mObsPositionWeight = config->get_float( "/firevision/omni/localizer/obstacle_position_weight" );

  // generate initial samples
  generateRandomSamples( mSampleCount );

  field_pos_t initialPos;
  initialPos.x = 0.0;
  initialPos.y = 0.0;
  initialPos.ori = 0.0;

  mOdometryPath.push_back( initialPos );
  mLastBestSample.position = initialPos;
  mLastBestSample.weight = 0.0;

#ifdef DEBUG_MCL_STATE
  calculatePose();
  dumpState( "mclstate-initial.jpg" );
#endif

#ifdef DEBUG_MCL_LOG
  mMCLLog.open( "mcllog.txt" );
#endif
}

/** Destructor. */
MCL::~ MCL()
{
  delete mField;
}

/**
  Update samples based on the last odometry data.
  @param movement The odometry data.
  @param pathLength The length of the moved path as reported by odometry.
*/
void MCL::predict( const field_pos_t &movement, float pathLength )
{
  const float noise = pathLength * 0.2;

#ifdef DEBUG_MCL_STATE
  field_pos_t absPos = mOdometryPath.back();
  absPos.x += ( cos(absPos.ori) * movement.x ) - ( sin(absPos.ori) * movement.y );
  absPos.y += ( sin(absPos.ori) * movement.x ) + ( cos(absPos.ori) * movement.y );
  absPos.ori += movement.ori;
  mOdometryPath.push_back( absPos );
#endif

  // ### add correct error model
  int droppedSamples = 0;
  for ( std::vector<mcl_sample_t>::iterator it = mSamples.begin(); it != mSamples.end(); ) {
    (*it).position.x += ( cos((*it).position.ori) * movement.x ) - ( sin((*it).position.ori) * movement.y )+ (mNormalDist( mRng ) * noise);
    (*it).position.y += ( sin((*it).position.ori) * movement.x ) + ( cos((*it).position.ori) * movement.y ) + (mNormalDist( mRng ) * noise);
    (*it).position.ori += movement.ori + (mNormalDist( mRng ) * movement.ori * 0.2);
    // drop samples that clearly moved outside
    if ( (std::abs((*it).position.x) + mOutOfFieldThreshold) > mField->totalWidth()/2.0 ||
         (std::abs((*it).position.y) + mOutOfFieldThreshold) > mField->totalHeight()/2.0 ) {
      ++droppedSamples;
      it = mSamples.erase( it );
    } else {
      clampToField( *it );
      ++it;
    }
  }

  if ( droppedSamples > 0 ) {
    std::cout << "Dropped " << droppedSamples << " samples that moved out of field." << std::endl;
    generateRandomSamples( droppedSamples );
  }

  if ( movement.x > 0.0 || movement.y > 0.0 || movement.ori != 0.0 )
    mNewOdometry = true;
  else
    mNewOdometry = false;
}

/**
  Prepare updating samples based on new sensor readings.
  Call this methods before any of the update() methods.
*/
void MCL::prepareUpdate()
{
#ifdef DEBUG_MCL_LOG
  mMCLLog << std::endl << "***************************** LOOP " << mLoopCount << " *********************" << std::endl;
#endif

  mField->updateDynamicObjects();
}

/**
  Update samples based on the last field line sensor readings.
  @param sensorHits The field line sensor readings.
*/
void MCL::update(const std::map< float, std::vector < polar_coord_t > > & sensorHits)
{
#ifdef DEBUG_MCL_UPDATE
  unsigned int width, height;
  float scale;
  unsigned char *buffer = createBuffer( width, height, scale );

  drawField( buffer, width, height, scale );
  mField->setDebugBuffer( buffer, width, height );
#endif

  for ( std::vector<mcl_sample_t>::iterator it = mSamples.begin(); it != mSamples.end(); ++it ) {
//     mMCLLog << "Raw sample: " << (*it).position << " weight: " << (*it).weight << std::endl;;
    (*it).weight = weightForPosition( (*it).position, sensorHits );
  }


#ifdef DEBUG_MCL_LOG
  sort( mSamples.rbegin(), mSamples.rend() );
  mLastBestSample = mSamples.front();

  for ( std::vector<mcl_sample_t>::iterator it = mSamples.begin(); it != mSamples.end(); ++it )
    mMCLLog << "Sample: " << (*it).position << " weight: " << (*it).weight << std::endl;;
#endif

#ifdef DEBUG_MCL_UPDATE
  mField->setDebugBuffer( 0 );
  char fileNameMCLUpdate[32];
  snprintf( fileNameMCLUpdate, 32, "mclupdate-%04d.jpg", mLoopCount );
  dumpBuffer( fileNameMCLUpdate, buffer, width, height );
#endif

#ifdef DEBUG_SENSOR_WEIGHTS
  char fileNameSensorWeights[32];
  snprintf( fileNameSensorWeights, 32, "sensor-weights-%04d.dat", mLoopCount );
  char fileNameObsWeights[32];
  snprintf( fileNameObsWeights, 32, "obs-weights-%04d.dat", mLoopCount );
  field_pos_t pos = mLastBestSample.position;
  pos.ori = 0.0;
  mField->dumpSensorProbabilities( pos, fileNameSensorWeights, fileNameObsWeights );
#endif

#ifdef DEBUG_POSITION_PROBABILITIES
  char fileNamePosProbs[32];
  snprintf( fileNamePosProbs, 32, "pos-probability-%04d.dat", mLoopCount );
  dumpPositionProbabilities( fileNamePosProbs, sensorHits );
#endif
}

/**
  Update samples based on detected ball positions.
  @param ballHits List of seen ball positions.
*/
void MCL::updateBall(const std::vector< f_point_t > & ballHits)
{
  if ( ballHits.empty() )
    return;

#ifdef DEBUG_MCL_LOG
  mMCLLog << "Got ball positions: ";
  copy( ballHits.begin(), ballHits.end(), ostream_iterator<f_point_t>( mMCLLog, " " ) );
  mMCLLog << std::endl;
#endif

  for ( std::vector<mcl_sample_t>::iterator it = mSamples.begin(); it != mSamples.end(); ++it ) {
    for ( std::vector<f_point_t>::const_iterator ballIt = ballHits.begin(); ballIt != ballHits.end(); ++ballIt )
      (*it).weight += mField->weightForBall( (*it).position, (*ballIt) );
  }

#ifdef DEBUG_MCL_LOG
  for ( std::vector<mcl_sample_t>::iterator it = mSamples.begin(); it != mSamples.end(); ++it )
    mMCLLog << "Sample with ball: " << (*it).position << " weight: " << (*it).weight << std::endl;;
#endif
}

/**
  Update samples based on detected obstacle positions.
  @param obstacleHits List of obstacles seen by our own sensors, as relative cartesian coordinates.
*/
void MCL::updateObstacles(const std::vector< obstacle_t > & obstacleHits)
{
#ifdef DEBUG_MCL_LOG
//   mMCLLog << "Got obstacle positions: ";
//   copy( obstacleHits.begin(), obstacleHits.end(), ostream_iterator<obstacle_t>( mMCLLog, " " ) );
//   mMCLLog << std::endl;
#endif

  std::vector<obstacle_t> expObstacles = mField->obstacles();
  for ( std::vector<mcl_sample_t>::iterator it = mSamples.begin(); it != mSamples.end(); ++it ) {

    float maxWeight = 0.0;
    float weight = 0.0;

    // get maximal possible weight
    for ( std::vector<obstacle_t>::const_iterator expIt = expObstacles.begin(); expIt != expObstacles.end(); ++expIt ) {
      maxWeight += mField->weightForObstacle( it->position, *expIt, *expIt );
    }
    // TODO consider our own position for maxWeight as well

    // get actual weight
    for ( std::vector<obstacle_t>::const_iterator obsIt = obstacleHits.begin(); obsIt != obstacleHits.end(); ++obsIt ) {
      float currentWeight = 0.0;
      for ( std::vector<obstacle_t>::const_iterator expIt = expObstacles.begin(); expIt != expObstacles.end(); ++expIt ) {
        // convert to global cartesian coordinates
        obstacle_t currentObs;
        currentObs.x = it->position.x + ( cosf(it->position.ori) * obsIt->x ) - ( sinf(it->position.ori) * obsIt->y );
        currentObs.y = it->position.y + ( sinf(it->position.ori) * obsIt->x ) + ( cosf(it->position.ori) * obsIt->y );
        currentObs.extent = obsIt->extent;
        currentObs.covariance = obsIt->covariance;
        currentWeight = std::max( mField->weightForObstacle( it->position, *expIt, currentObs ), currentWeight );
      }
      weight += currentWeight;
    }

    // Others observe us as obstacle as well
    obstacle_t myself;
    myself.x = it->position.x;
    myself.y = it->position.y;
    myself.extent = 0.48;
    // TODO covariance?!?!?
    float myWeight = 0.0;
    for ( std::vector<obstacle_t>::const_iterator expIt = expObstacles.begin(); expIt != expObstacles.end(); ++expIt )
      myWeight = std::max( myWeight, mField->weightForObstacle( it->position, *expIt, myself ) );
    weight += myWeight;

    (*it).weight += ( weight / maxWeight ) * mObsPositionWeight;
  }

#ifdef DEBUG_MCL_LOG
  for ( std::vector<mcl_sample_t>::iterator it = mSamples.begin(); it != mSamples.end(); ++it )
    mMCLLog << "Sample with obstacles: " << (*it).position << " weight: " << (*it).weight << std::endl;;
#endif
}

/**
  Calculate the weight for the given position @p pos and
  the given sensor readings @p sensorHits.
  @param pos The current position.
  @param sensorHits The sensor readings.
*/
float MCL::weightForPosition( const field_pos_t &pos,
                              const std::map< float, std::vector < polar_coord_t > > &sensorHits ) const
{
  float maxWeight = 0.0;
  float sensorWeight = 0.0;
  int totalExpected = 0;
  typedef std::map<float, std::vector<polar_coord_t> > HitMap;
  for ( HitMap::const_iterator rayIt = sensorHits.begin(); rayIt != sensorHits.end(); ++rayIt ) {
    std::vector<float> expectedHits = mField->findIntersections( pos, (*rayIt).first );
    if ( expectedHits.empty() ) {
      // calculate penalty for unexpected hits
      float prevHit = -mUnexpectedPenaltyMinDistance;
      for ( std::vector<polar_coord_t>::const_iterator it = (*rayIt).second.begin(); it != (*rayIt).second.end(); ++it ) {
        if ( (*it).r > prevHit + mUnexpectedPenaltyMinDistance )
          sensorWeight += mUnexpectedPenalty;
        prevHit = (*it).r;
      }
      continue;
    }
//     mMCLLog << "expected: ";
//     copy( expectedHits.begin(), expectedHits.end(), ostream_iterator<float>( mMCLLog, ", " ) );
//     mMCLLog << std::endl << "actual: ";
//     copy( rayIt->second.begin(), rayIt->second.end(), ostream_iterator<polar_coord_t>( mMCLLog, ", " ) );
//     mMCLLog << std::endl;
    totalExpected += expectedHits.size();
    for ( std::vector<float>::const_iterator expHitIt = expectedHits.begin(); expHitIt != expectedHits.end(); ++expHitIt )
      maxWeight += mField->weightForDistance( *expHitIt, *expHitIt );

    // find the closest sensor reading to the expected hits
    // make sure we only count the best sensor reading per expected hit to avoid
    // skewing the results for parallel lines (real parallel lines as well as those caused
    // by cam vibrations)
    std::map< float, std::pair<float,float> > bestSensorWeights;
    sort( expectedHits.begin(), expectedHits.end() );
    float penalty = 0.0;
    for (std::vector<polar_coord_t>::const_iterator hitIt = (*rayIt).second.begin(); hitIt != (*rayIt).second.end(); ++hitIt ) {
      const float closestExp = find_closest( expectedHits.begin(), expectedHits.end(), (*hitIt).r );
      // only keep the best weight for this expected hit
      if ( bestSensorWeights.find( closestExp ) != bestSensorWeights.end() ) {
        const std::pair<float, float> prevWeight = bestSensorWeights[ closestExp ];
        const float currentWeight = mField->weightForDistance( closestExp, (*hitIt).r );
        if ( currentWeight > prevWeight.first )
          bestSensorWeights[ closestExp ] = std::make_pair( currentWeight, (*hitIt).r );
        if ( std::abs( (*hitIt).r - prevWeight.second ) > mUnexpectedPenaltyMinDistance )
          penalty += mUnexpectedPenalty;
      } else {
        bestSensorWeights[ closestExp ] = std::make_pair( mField->weightForDistance( closestExp, (*hitIt).r ), (*hitIt).r );
      }
    }

    // sum up all weights for this ray
    for ( std::map<float, std::pair<float, float> >::const_iterator it = bestSensorWeights.begin(); it != bestSensorWeights.end(); ++it )
      sensorWeight += (*it).second.first;
    sensorWeight += penalty;
  }
  sensorWeight = std::max( sensorWeight, 0.0f );
  if ( maxWeight > 0.0 )
    return (sensorWeight / maxWeight) * SENSOR_COUNT_WEIGHT( totalExpected );
  return 0.0;
}

/** Evaluate samples. */
void MCL::resample()
{
  sort( mSamples.rbegin(), mSamples.rend() );
  mLastBestSample = mSamples.front();

  if ( !mNewOdometry ) {
   std::vector<mcl_sample_t>::reverse_iterator it = upper_bound( mSamples.rbegin(), mSamples.rend(), mIdleRegenerationThreshold );
    if ( it == mSamples.rend() ) {
      std::cout << "WTF? all samples bad?" << std::endl;
      mSamples.clear();
      generateRandomSamples( mSampleCount );
    } else if ( it != mSamples.rend() ) {
      const int index = it - mSamples.rbegin();
      std::cout << "removing samples after: " << (*it).weight << " and generating " << index << " new samples" << std::endl;
      mSamples.resize( mSampleCount - index );
      generateRandomSamples( index );
    } else {
      std::cout << "skipping resampling - no movement" << std::endl;
    }
    return;
  }
  mNewOdometry = false;

  // accumulate weights
  for ( unsigned int i = 1; i < mSamples.size(); ++i )
    mSamples[i].weight = RESAMPLE_ADJUST(mSamples[i].weight) + mSamples[i - 1].weight;

#ifdef DEBUG_MCL_LOG
  mMCLLog << std::endl << "*** RESAMPLE ***" << std::endl;
  for (std::vector<mcl_sample_t>::iterator it = mSamples.begin(); it != mSamples.end(); ++it ) {
    mMCLLog << (*it).weight << ", ";
  }
  mMCLLog << std::endl;
#endif

  if ( mSamples.back().weight == 0.0 ) {
    std::cout << "WTF? no weights - regenerating samples" << std::endl;
    mSamples.clear();
    generateRandomSamples( mSampleCount );
    return;
  }
  sort( mSamples.begin(), mSamples.end() );

  // draw new samples
 std::vector<mcl_sample_t> newSamples;
  for ( unsigned int i = 0; i < (mSampleCount * (1 - mFreeSampleRatio)); ++i ) {
    float random = mUniformDist() * mSamples.back().weight;
   std::vector<mcl_sample_t>::iterator it = lower_bound( mSamples.begin(), mSamples.end(), random );
    mcl_sample_t sample = *it;
    sample.weight = 1.0;

    // ### add noise?
    sample.position.x += mNormalDist( mRng ) * 0.1;
    sample.position.y += mNormalDist( mRng ) * 0.1;
    sample.position.ori += mNormalDist( mRng ) * 0.1;
    clampToField( sample );

    newSamples.push_back( sample );
  }
  mSamples = newSamples;
  generateRandomSamples( (int)(mSampleCount * mFreeSampleRatio) ); // a few random samples to deal with kidnapped robot situations
}

/**
  Calculate the current pose estimate and variance.
*/
void MCL::calculatePose()
{
  normalizeSamples();

  mMean.x = 0.0;
  mMean.y = 0.0;
  mMean.ori = 0.0;
  for (std::vector<mcl_sample_t>::const_iterator it = mSamples.begin(); it != mSamples.end(); ++it ) {
    mMean.x += (*it).position.x * (*it).weight;
    mMean.y += (*it).position.y * (*it).weight;
    mMean.ori += (*it).position.ori * (*it).weight;
  }

  mVariance.x = 0.0;
  mVariance.y = 0.0;
  mVariance.ori = 0.0;
  for (std::vector<mcl_sample_t>::const_iterator it = mSamples.begin(); it != mSamples.end(); ++it ) {
    mVariance.x += ((*it).position.x - mMean.x) * ((*it).position.x - mMean.x) * (*it).weight;
    mVariance.y += ((*it).position.y - mMean.y) * ((*it).position.y - mMean.y) * (*it).weight;
    mVariance.ori += ((*it).position.ori - mMean.ori) * ((*it).position.ori - mMean.ori) * (*it).weight;
  }

  if ( mVariance.x > mVarianceLimit.x || mVariance.y > mVarianceLimit.y || mVariance.ori > mVarianceLimit.ori )
    mPose = mLastBestSample.position;
  else
    mPose = mMean;
}

/**
  Returns the current pose estimae.
*/
field_pos_t MCL::pose() const
{
  return mPose;
}

/**
  Returns the variance of the current pose estimate.
*/
field_pos_t MCL::variance() const
{
  return mVariance;
}

/**
  Normalize samples.
*/
void MCL::normalizeSamples()
{
  float p = 0.0;
  for (std::vector<mcl_sample_t>::const_iterator it = mSamples.begin(); it != mSamples.end(); ++it )
    p += (*it).weight;
  if ( p != 1.0 && p != 0.0 ) {
    for (std::vector<mcl_sample_t>::iterator it = mSamples.begin(); it != mSamples.end(); ++it )
      (*it).weight /= p;
  }
}


#define mapToImageX(x) ((unsigned int)(scale * ((x) + mField->totalWidth()/2.0)))
#define mapToImageY(y) ((unsigned int)(scale * ((y) + mField->totalHeight()/2.0)))

/**
  Dump current state into image file.
  @param filename The filename.
*/
void MCL::dumpState(const char * filename)
{
  field_pos_t estimate = pose();
  mPath.push_back( estimate );
  std::cout << "MCL: esitmated position: " << estimate << std::endl;
  std::cout << "MCL: mean: " << mMean << " variance: " << mVariance << std::endl;
  std::cout << "MCL: top sample is: " << mLastBestSample.position << " p: " << mLastBestSample.weight <<std::endl;

  unsigned int width, height;
  float scale;
  unsigned char *buffer = createBuffer( width, height, scale );

  Drawer debugDrawer;
  debugDrawer.setBuffer( buffer, width, height );

  drawField( buffer, width, height, scale );

  // odometry path
  debugDrawer.setColor( 128, 32, 255 );
  if ( mOdometryPath.size() >= 2 ) {
   std::vector<field_pos_t>::const_iterator currentIt = mOdometryPath.begin();
   std::vector<field_pos_t>::const_iterator prevIt = currentIt++;
    for ( ; currentIt != mOdometryPath.end(); ++currentIt, ++prevIt ) {
      debugDrawer.drawLine( mapToImageX( prevIt->x ), mapToImageY( prevIt->y ),
                            mapToImageX( currentIt->x ), mapToImageY( currentIt->y ) );
    }
  }

  // previous path
  debugDrawer.setColor( 96, 128, 128 );
  if ( mPath.size() >= 2 ) {
   std::vector<field_pos_t>::const_iterator currentIt = mPath.begin();
   std::vector<field_pos_t>::const_iterator prevIt = currentIt++;
    for ( ; currentIt != mPath.end(); ++currentIt, ++prevIt ) {
      debugDrawer.drawLine( mapToImageX( prevIt->x ), mapToImageY( prevIt->y ),
                            mapToImageX( currentIt->x ), mapToImageY( currentIt->y ) );
    }
  }

  // top sample
  debugDrawer.setColor( 128, 255, 0 );
  drawPosition( &debugDrawer, scale, mLastBestSample.position );

  // mean + variance
  debugDrawer.setColor( 128, 255, 255 );
  drawPosition( &debugDrawer, scale, mMean );
  const float variR = std::max( mVariance.x, mVariance.y );
  debugDrawer.drawCircle( mapToImageX(mMean.x), mapToImageY(mMean.y),
                          (unsigned int)( variR * scale ) );
  float variX = variR * cos( mMean.ori + mVariance.ori ) + mMean.x;
  float variY = variR * sin( mMean.ori + mVariance.ori ) + mMean.y;
  debugDrawer.drawLine( mapToImageX(mMean.x), mapToImageY(mMean.y), mapToImageX(variX), mapToImageY(variY) );
  variX = variR * cos( mMean.ori - mVariance.ori ) + mMean.x;
  variY = variR * sin( mMean.ori - mVariance.ori ) + mMean.y;
  debugDrawer.drawLine( mapToImageX(mMean.x), mapToImageY(mMean.y), mapToImageX(variX), mapToImageY(variY) );

  // official estimate
  debugDrawer.setColor( 128, 0, 128 );
  drawPosition( &debugDrawer, scale, estimate );


  dumpBuffer( filename, buffer, width, height );
}

/**
  Create a debug image buffer.
  @param width The image width.
  @param height The image height.
  @param scale The image scale.
*/
unsigned char* MCL::createBuffer(unsigned int & width, unsigned int & height, float & scale)
{
  width = DEBUG_IMAGE_SIZE;
  height = (unsigned int)(width * mField->totalHeight() / mField->totalWidth());
  scale = width / mField->totalWidth();

  unsigned char *buffer;
  const unsigned int debugBufferSize = colorspace_buffer_size( YUV422_PLANAR, width, height );
  buffer = new unsigned char[debugBufferSize];
  memset( buffer, 0, debugBufferSize );
  return buffer;
}

/**
  Write debug buffer to a file.
  @param filename The filename.
  @param buffer The debug image buffer.
  @param width The image width.
  @param height The image height.
*/
void MCL::dumpBuffer(const char * filename, unsigned char *buffer, unsigned int width, unsigned int height)
{
  JpegWriter writer;
  writer.set_dimensions( width, height );
  writer.set_buffer( YUV422_PLANAR, buffer );
  writer.set_filename( filename );
  writer.write();

  delete[] buffer;
}

/**
  Draw a bot marker into the debug image.
  @param drawer The drawer.
  @param scale The image scale.
  @param pos The robot position.
*/
void MCL::drawPosition(Drawer * drawer, float scale, const field_pos_t & pos)
{
  drawer->drawCircle( mapToImageX(pos.x), mapToImageY(pos.y), (unsigned int)(0.25 * scale) );
  float oriX = 0.25 * cos( pos.ori ) + pos.x;
  float oriY = 0.25 * sin( pos.ori ) + pos.y;
  drawer->drawLine( mapToImageX(pos.x), mapToImageY(pos.y), mapToImageX(oriX), mapToImageY(oriY) );
}

/**
  Draw the field including all samples into an image.
  @param buffer The debug image buffer.
  @param width The image width.
  @param height The image height.
  @param scale The image scale.
*/
void MCL::drawField(unsigned char * buffer, unsigned int width, unsigned int height, float scale)
{
  mField->setDebugBuffer( buffer, width, height );
  mField->drawField();
  mField->setDebugBuffer( 0 );

  Drawer debugDrawer;
  debugDrawer.setBuffer( buffer, width, height );

  for (std::vector<mcl_sample_t>::const_iterator it = mSamples.begin(); it != mSamples.end(); ++it ) {
    debugDrawer.setColor( (unsigned int)(255.0 * (*it).weight),
                           128 - (unsigned int)(128.0 * (*it).weight),
                           128 + (unsigned int)(127.0 * (*it).weight) );
    debugDrawer.drawCircle( mapToImageX((*it).position.x), mapToImageY((*it).position.y), (unsigned int)(0.25 * scale) );
    float oriX = 0.25 * cos( (*it).position.ori ) + (*it).position.x;
    float oriY = 0.25 * sin( (*it).position.ori ) + (*it).position.y;
    debugDrawer.drawLine( mapToImageX((*it).position.x), mapToImageY((*it).position.y),
                          mapToImageX(oriX), mapToImageY(oriY) );
  }
}

/**
  Restricts @p sample to the field and normalizes its orientation.
  @param sample The sample to normalize.
*/
void MCL::clampToField(mcl_sample_t & sample) const
{
  // ### should not happen
  if ( isnan(sample.position.x) )
    sample.position.x = 0.0;
  if ( isnan(sample.position.y) )
    sample.position.y = 0.0;
  if ( isnan(sample.position.ori) )
    sample.position.ori = 0.0;

  sample.position.x = std::min( std::max( sample.position.x, (float)(-mField->totalWidth() / 2.0) ),
				(float)(mField->totalWidth() / 2.0) );
  sample.position.y = std::min( std::max( sample.position.y, (float)(-mField->totalHeight() / 2.0) ),
				(float)(mField->totalHeight() / 2.0) );
  sample.position.ori -= ((int)(sample.position.ori / (2.0*M_PI))) * 2.0 * M_PI;
  if ( sample.position.ori < 0 )
    sample.position.ori += 2.0 * M_PI;
}

/**
  Generate uniformly distributed samples.
  @param count The number of samples to generate.
*/
void MCL::generateRandomSamples(int count)
{
  for ( int i = 0; i < count; ++i ) {
    mcl_sample_t sample;
    sample.weight = 1.0/mSampleCount;
    sample.position.x = (mUniformDist() * mField->totalWidth()) - (mField->totalWidth() / 2.0);
    sample.position.y = (mUniformDist() * mField->totalHeight()) - (mField->totalHeight() / 2.0);
    sample.position.ori = mUniformDist() * M_PI * 2.0;
    mSamples.push_back( sample );
  }
}

/**
  Generate normally distributed samples.
  @param count The number of samples to generate.
  @param position The mean of the sample positions.
  @param variance The variance of the generated samples.
*/
void MCL::generateRandomSamples(int count, const field_pos_t & position, float variance)
{
  for ( int i = 0; i < count; ++i ) {
    mcl_sample_t sample;
    sample.weight = 1.0/mSampleCount;
    sample.position.x = position.x + (mNormalDist( mRng ) * variance);
    sample.position.y = position.y + (mNormalDist( mRng ) * variance);
    sample.position.ori = position.ori + (mNormalDist( mRng ) * variance);
    clampToField( sample );
    mSamples.push_back( sample );
  }
}

/**
  Write position probabilities to gnuplot file for debugging.
  @param filename The destination file.
  @param sensorHits The sensor readings.
*/
void MCL::dumpPositionProbabilities( const char * filename,
                                     const std::map< float, std::vector < polar_coord_t > > & sensorHits ) const
{
  std::ofstream s( filename );
  for ( float x = -mField->totalWidth()/2.0; x < mField->totalWidth()/2.0; x += 0.1 ) {
    for ( float y = -mField->totalHeight()/2.0; y < mField->totalHeight()/2.0; y += 0.1 ) {
      float weight = 0.0;
      float maxPhi = 0.0;
      for ( float phi = 0.0; phi < M_PI * 2.0; phi += 0.1 ) {
        field_pos_t pos;
        pos.x = x;
        pos.y = y;
        pos.ori = phi;
        const float w = weightForPosition( pos, sensorHits );
        if ( w > weight ) {
          weight = w;
          maxPhi = phi;
        }
      }
      s << x << " " << y << " " << weight << " " << maxPhi << std::endl;
    }
    s << std::endl;
  }
}
