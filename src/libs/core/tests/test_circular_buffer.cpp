/***************************************************************************
 *  test_circular_buffer.cpp - CircularBuffer Unit Test
 *
 *  Created: Fri Aug 15 16:27:42 2014
 *  Copyright  2014  Till Hofmann
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

#include <gtest/gtest.h>

#include <core/utils/circular_buffer.h>
#include <stdexcept>

using namespace fawkes;


TEST(CircularBufferTest, ElementAccess)
{
  CircularBuffer<int> buffer(1000);
  for (int i = 0; i < 1000; i++) {
    buffer.push_back(i);
  }
  for (int i = 0; i < 1000; i++) {
   ASSERT_EQ(i, buffer[i]);
   ASSERT_EQ(i, buffer.at(i));
  }
}

TEST(CircularBufferTest, ElementDeletion)
{
  CircularBuffer<int> buffer(1);
  buffer.push_back(1);
  buffer.push_back(2);
  ASSERT_EQ(1, buffer.size());
  ASSERT_EQ(2, buffer[0]);
}

TEST(CircularBufferTest, OutOfMaxRange)
{
  CircularBuffer<int> buffer(1);
  int i;
  ASSERT_NO_THROW(i = buffer[1]);
  ASSERT_THROW(i = buffer.at(1), std::out_of_range);
}

TEST(CircularBufferTest, OutOfRange)
{
  CircularBuffer<int> buffer(2);
  buffer.push_back(1);
  int i;
  ASSERT_NO_THROW(i = buffer[1]);
  ASSERT_THROW(i = buffer.at(1), std::out_of_range);
}


TEST(CircularBufferTest, ConstValues)
{
  CircularBuffer<int> buffer(5);
  // buffer[0] = 2; // won't compile
  CircularBuffer<int>::iterator it = buffer.begin();
  // *it = 2; // won't compile

}

TEST(CircularBufferTest, CopyConstructor)
{
  CircularBuffer<int> b1(5);
  b1.push_back(1);
  b1.push_back(2);
  CircularBuffer<int> b2(b1);
  ASSERT_EQ(5, b2.get_max_size());
  ASSERT_EQ(1, b2[0]);
  ASSERT_EQ(2, b2[1]);
  b2.push_back(3);
  ASSERT_EQ(3, b2[2]);
  ASSERT_EQ(1, b1[0]);
  ASSERT_EQ(2, b1[1]);
  ASSERT_THROW(b1.at(2), std::out_of_range);
}
