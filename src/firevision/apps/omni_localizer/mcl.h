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

#ifndef FVOMNILOCALIZER_MCL_H
#define FVOMNILOCALIZER_MCL_H

#include <fvutils/base/types.h>

#include <boost/random.hpp>

#include <map>
#include <vector>
#include <fstream>

class Drawer;
class Field;
class Configuration;

/** Represents a MCL sample. */
struct mcl_sample_t {
  /** The position of the sample. */
  field_pos_t position;
  /** The weight of the sample. */
  float weight;
};

class MCL
{
  public:
    MCL( Configuration *config );
    ~MCL();

    void predict( const field_pos_t &movement, float pathLength );
    void update( const std::map< float, std::vector<polar_coord_t> > &sensorHits );
    void resample();

    field_pos_t currentEstimate();
    field_pos_t currentVariance() const;

    void dumpState( const char *filename );

    /** Debug hack, remove at some point. */
    int mLoopCount; // ### DEBUG HACK

  private:
    void normalizeSamples();
    void clampToField( mcl_sample_t &sample ) const;
    void generateRandomSamples( int count );
    void generateRandomSamples( int count, const field_pos_t &position, float variance );

    float weightForPosition( const field_pos_t &pos,
                             const std::map< float, std::vector<polar_coord_t> > &sensorHits ) const;

    unsigned char* createBuffer( unsigned int &width, unsigned int &height, float &scale );
    void dumpBuffer( const char *filename, unsigned char *buffer, unsigned int width, unsigned int height );
    void drawPosition( Drawer* drawer, float scale, const field_pos_t &pos );
    void drawField( unsigned char *buffer, unsigned int width, unsigned int height, float scale );
    void dumpPositionProbabilities( const char *filename,
                                    const std::map< float, std::vector<polar_coord_t> > &sensorHits ) const;

  private:
    std::vector<mcl_sample_t> mSamples;
    Field* mField;
    bool mNewOdometry;
    mcl_sample_t mLastBestSample;

    field_pos_t mMean;
    field_pos_t mVariance;

    unsigned int mSampleCount;
    float mFreeSampleRatio;
    float mIdleRegenerationThreshold;
    field_pos_t mVarianceLimit;
    float mUnexpectedPenalty;
    float mUnexpectedPenaltyMinDistance;

    // debug
    std::vector<field_pos_t> mPath;
    std::vector<field_pos_t> mOdometryPath;
    mutable std::ofstream mMCLLog;

    boost::lagged_fibonacci9689 mRng;
    boost::uniform_01<boost::lagged_fibonacci9689, float> mUniformDist;
    boost::normal_distribution<float> mNormalDist;
};

#endif
