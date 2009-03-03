
/***************************************************************************
 *  vector.cpp - Column Vector
 *
 *  Created: Wed April 02 14:11:03 2008
 *  Copyright  2008  Daniel Beck
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

#include "vector.h"
#include <exception>
#include <core/exceptions/software.h>
#include <cstdlib>
#include <cstdio>

namespace fawkes {

/** @class Vector geometry/vector.h
 * A simple column vector.
 * @author Daniel Beck
 */

/** Constructor.
 * @param size the dimension of the vector
 * @param data pointer to a float array
 * @param manage_memory if true, the Vector will manage its memory on
 * its own, else it will not allocate new memory but works with the
 * provided array
 */
Vector::Vector(unsigned int size, float* data, bool manage_memory)
{
  m_size = size;
  m_manage_memory = manage_memory;

  if (m_manage_memory)
    {
      m_data = new float[m_size];

      for (unsigned int i = 0; i < m_size; ++i)
	{
	  if (data)
	    { m_data[i] = data[i]; }
	  else
	    { m_data[i] = 0.0; }
	}
    }
  else
    {
      m_data = data;
    }
}

/** Copy constructor.
 * @param v another Vector
 */
Vector::Vector(const Vector& v)
{
  m_size = v.m_size;
  m_manage_memory = true;
  m_data = new float[m_size];

  for (unsigned int i = 0; i < m_size; ++i)
    {
      m_data[i] = v.m_data[i];
    }
}

/** Destructor. */
Vector::~Vector()
{
  if (m_manage_memory)
    {
      delete[] m_data;
    }
}

/** Get the number of elements.
 * @return number of elements
 */
unsigned int
Vector::size() const
{
  return m_size;
}

/** Set a new size.
 * @param size the new size
 */
void
Vector::set_size(unsigned int size)
{
  float* t = new float[size];

  unsigned int i = 0;
  while( i < size && i < m_size)
    {
      t[i] = m_data[i];
      ++i;
    }

  m_size = size;

  if (m_manage_memory) //I'm not supposed to delete foreign buffers
    { delete[] m_data; }
  else
    { m_manage_memory = true;}
  
  m_data = t;
}

/** Get pointer to the internal data container.
 * @return pointer to the internal data container
 */
float*
Vector::data_ptr() const
{
  return m_data;
}

/** Get a certain element.
 * @param d index of the requested element
 * @return element at position d
 */
float
Vector::get(unsigned int d) const
{
  if (m_size <= d)
    { return 0.0; }

  return m_data[d];
}

/** Get a reference to a certain element.
 * @param d index of the requested element
 * @return reference to element at position d
 */
float&
Vector::get(unsigned int d)
{
  if (m_size <= d)
    {
      printf("This column vector has %u elements -- element %u not "
	     "available", m_size, d);
      throw std::exception();
    }

  return m_data[d];
}

/** Set a certain element.
 * @param d index of the element
 * @param f the new value
 */
void
Vector::set(unsigned int d, float f)
{
  if (m_size <= d)
    {
      printf("This column vector has %u elements -- element %u not "
	     "available", m_size, d);
      throw std::exception();
    }

  m_data[d] = f;
}

/** Convenience getter to obtain the first element.
 * @return the first element
 */
float
Vector::x() const
{
  return get(0);
}

/** Convenience getter to obtain a reference to the first element.
 * @return reference to the first element
 */
float&
Vector::x()
{
  float& ret = get(0);
  return ret;
}

/** Convenience setter to set the first element.
 * @param x the new value of the first element
 */
void
Vector::x(float x)
{
  set(0, x);
}

/** Convenience getter to obtain the second element.
 * @return the second element
 */
float
Vector::y() const
{
  return get(1);
}

/** Convenience getter to obtain a reference to the second element.
 * @return reference to the second element
 */
float&
Vector::y()
{
  float& ret = get(1);
  return ret;
}

/** Convenience setter to set the second element.
 * @param y the new value of the second element
 */
void
Vector::y(float y)
{
  set(1, y);
}

/** Convenience getter to obtain the third element.
 * @return the third element
 */
float
Vector::z() const
{
  return get(2);
}

/** Convenience getter to obtain a reference to the third element.
 * @return reference to the third element
 */
float&
Vector::z()
{
  float& ret = get(2);
  return ret;
}

/** Convenience setter to set the third element.
 * @param z the new value of the third element
 */
void
Vector::z(float z)
{
  set(2, z);
}

/** Access operator.
 * @param d index of the requested element
 * @return the value at the given position
 */
float
Vector::operator[](unsigned int d) const
{
  if (m_size <= d)
    { return 0.0; }

  return m_data[d];
}

/** Access operator.
 * @param d index of the requested element
 * @return reference to the value at the given position
 */
float&
Vector::operator[](unsigned int d)
{
  if (m_size <= d)
    {
      printf("This column vector has %u elements -- element %u not "
	     "available", m_size, d);
      throw std::exception();
    }

  return m_data[d];
}

/** Multiply the vector with a scalar.
 * @param f the scalar
 * @return scaled vector
 */
Vector
Vector::operator*(const float& f) const
{
  Vector result(m_size, m_data);

  for (unsigned int i = 0; i < m_size; ++i)
    { result.m_data[i] *= f; }

  return result;
}

/** In-place scalar multiplication.
 * @param f the scalar
 * @return reference to the scaled vector
 */
Vector&
Vector::operator*=(const float& f)
{
  for (unsigned int i = 0; i < m_size; ++i)
    { m_data[i] *= f; }

  return *this;
}

/** Divide every element of the vector by a scalar.
 * @param f the scalar
 * @return scaled vector
 */
Vector
Vector::operator/(const float& f) const
{
  Vector result(m_size, m_data);

  for (unsigned int i = 0; i < m_size; ++i)
    { result.m_data[i] /= f; }

  return result;
}

/** In-place scalar division.
 * @param f the scalar
 * @return reference to the scaled vector
 */
Vector&
Vector::operator/=(const float& f)
{
  for (unsigned int i = 0; i < m_size; ++i)
    { m_data[i] /= f; }

  return *this;
}

/** Adds two vectors.
 * @param cv the vector to be added
 * @return sum vector
 */
Vector
Vector::operator+(const Vector& cv) const
{
  if (m_size != cv.size()) throw fawkes::TypeMismatchException("The two vectors have to be of equal size");

  Vector result(m_size, m_data);

  for (unsigned int i = 0; i < m_size; ++i)
    {
      result.m_data[i] += cv[i];
    }

  return result;
}

/** In-place vector addition.
 * @param cv the vector to be added
 * @return reference to the sum vector
 */
Vector&
Vector::operator+=(const Vector& cv)
{
  if (m_size != cv.size()) throw fawkes::TypeMismatchException("The two vectors have to be of equal size");

  for (unsigned int i = 0; i < m_size; ++i)
    {
      m_data[i] += cv[i];
    }

  return *this;
}

/** Substract two vectors.
 * @param cv the vector to be substracted
 * @return diff vector
 */
Vector
Vector::operator-(const Vector& cv) const
{
  if (m_size != cv.size()) throw fawkes::TypeMismatchException("The two vectors have to be of equal size");

  Vector result(m_size, m_data);

  for (unsigned int i = 0; i < m_size; ++i)
    {
      result.m_data[i] -= cv[i];
    }

  return result;
}

/** In-place vector substraction.
 * @param cv the vector to be substracted
 * @return reference to the diff vector
 */
Vector&
Vector::operator-=(const Vector& cv)
{
  if (m_size != cv.size()) throw fawkes::TypeMismatchException("The two vectors have to be of equal size");

  for (unsigned int i = 0; i < m_size; ++i)
    {
      m_data[i] -= cv[i];
    }

  return *this;
}

/** Assignment operator.
 * @param v the rhs vector
 * @return reference to the lhs vector
 */
Vector&
Vector::operator=(const Vector& v)
{
  if (m_size != v.m_size)
    {
      if (m_manage_memory)
	{ delete[] m_data; }

      m_size = v.m_size;
      m_manage_memory = true;

      m_data = new float[m_size];
    }

  for (unsigned int i = 0; i < m_size; ++i)
    { m_data[i] =  v.m_data[i]; }

  return *this;
}

/** Comparison operator.
 * @param v the other vector
 * @return true, if both vectors are equal
 */
bool
Vector::operator==(const Vector& v)
{
  if (m_size != v.m_size)
    { return false; }

  for (unsigned int i = 0; i < m_size; ++i)
    {
      if (m_data[i] != v.m_data[i])
	{ return false; }
    }

  return true;
}


/** Prints the vector data to standard out.
 * @param name a string that is printed prior to the vector data
 */
void
Vector::print_info(const char* name) const
{
  if (name)
    { printf("%s: [ ", name); }
  else
    { printf("[ "); }

  for (unsigned int i = 0; i < m_size; ++i)
    {
      printf("%f ", get(i));
    }
  printf("]T\n");
}

/** Calculates the dot product of two vectors.
 * @param v the rhs Vector
 * @return the scalar product
 */
float
Vector::operator*(const Vector& v) const
{
  float res = 0;

  for (unsigned int i = 0; i < m_size; ++i)
    res += this->get(i) * v.get(i);

  return res;
}


/**
 * Appends the components of the Vector to the ostream
 * @param stream the input stream
 * @param v the vector to be appended
 * @return the resulting stream
 */
std::ostream&
operator<<(std::ostream& stream, const Vector &v)
{
  stream << "[";
  
  for (unsigned int i = 0; i < v.m_size; ++i)
    {
      stream << v.get(i);
      
      if (i + 1 < v.m_size)
	stream << ",";
    }
  
  return stream << "]";
}

} // end namespace fawkes

