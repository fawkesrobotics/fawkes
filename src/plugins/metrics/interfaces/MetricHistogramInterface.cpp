
/***************************************************************************
 *  MetricHistogramInterface.cpp - Fawkes BlackBoard Interface - MetricHistogramInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2017  Tim Niemueller
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

#include <interfaces/MetricHistogramInterface.h>

#include <core/exceptions/software.h>

#include <map>
#include <string>
#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class MetricHistogramInterface <interfaces/MetricHistogramInterface.h>
 * MetricHistogramInterface Fawkes BlackBoard Interface.
 * 
		  A histogram samples observations (usually things like request
		  durations or response sizes) and counts them in configurable
		  buckets. It also provides a sum of all observed values.

		  A histogram with a base metric name of *basename* exposes
		  multiple time series during a scrape:

		  - cumulative counters for the observation buckets, exposed as
		    *basename*_bucket{le="*upper inclusive bound*"}

		  - the total sum of all observed values, exposed as *basename*_sum

		  - the count of events that have been observed, exposed as
		    *basename*_count (identical to *basename*_bucket{le="+Inf"}
		    above)

		  When operating on buckets, remember that the histogram is
		  cumulative. See histograms and summaries for details of
		  histogram usage and differences to summaries.

		  The interface supports histograms of up to 16 buckets.
	  
 * @ingroup FawkesInterfaces
 */



/** Constructor */
MetricHistogramInterface::MetricHistogramInterface() : Interface()
{
  data_size = sizeof(MetricHistogramInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (MetricHistogramInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_STRING, "labels", 512, data->labels);
  add_fieldinfo(IFT_UINT64, "sample_count", 1, &data->sample_count);
  add_fieldinfo(IFT_DOUBLE, "sample_sum", 1, &data->sample_sum);
  add_fieldinfo(IFT_UINT32, "bucket_count", 1, &data->bucket_count);
  add_fieldinfo(IFT_UINT64, "bucket_cumulative_count", 16, &data->bucket_cumulative_count);
  add_fieldinfo(IFT_DOUBLE, "bucket_upper_bound", 16, &data->bucket_upper_bound);
  unsigned char tmp_hash[] = {0xe5, 0xbf, 0x9, 0xe1, 0xa7, 0xcc, 0x57, 0xe4, 0x87, 0xe4, 0x97, 0x5, 0x32, 0x25, 0xa9, 0x2b};
  set_hash(tmp_hash);
}

/** Destructor */
MetricHistogramInterface::~MetricHistogramInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get labels value.
 * 
		  Labels must be given as key-value pairs of the form
		  "key=value[,key=value...]".
	  
 * @return labels value
 */
char *
MetricHistogramInterface::labels() const
{
  return data->labels;
}

/** Get maximum length of labels value.
 * @return length of labels value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MetricHistogramInterface::maxlenof_labels() const
{
  return 512;
}

/** Set labels value.
 * 
		  Labels must be given as key-value pairs of the form
		  "key=value[,key=value...]".
	  
 * @param new_labels new labels value
 */
void
MetricHistogramInterface::set_labels(const char * new_labels)
{
  strncpy(data->labels, new_labels, sizeof(data->labels));
  data_changed = true;
}

/** Get sample_count value.
 * 
		  The number of all samples.
    
 * @return sample_count value
 */
uint64_t
MetricHistogramInterface::sample_count() const
{
  return data->sample_count;
}

/** Get maximum length of sample_count value.
 * @return length of sample_count value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MetricHistogramInterface::maxlenof_sample_count() const
{
  return 1;
}

/** Set sample_count value.
 * 
		  The number of all samples.
    
 * @param new_sample_count new sample_count value
 */
void
MetricHistogramInterface::set_sample_count(const uint64_t new_sample_count)
{
  data->sample_count = new_sample_count;
  data_changed = true;
}

/** Get sample_sum value.
 * 
		  The sum of all samples.
	  
 * @return sample_sum value
 */
double
MetricHistogramInterface::sample_sum() const
{
  return data->sample_sum;
}

/** Get maximum length of sample_sum value.
 * @return length of sample_sum value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MetricHistogramInterface::maxlenof_sample_sum() const
{
  return 1;
}

/** Set sample_sum value.
 * 
		  The sum of all samples.
	  
 * @param new_sample_sum new sample_sum value
 */
void
MetricHistogramInterface::set_sample_sum(const double new_sample_sum)
{
  data->sample_sum = new_sample_sum;
  data_changed = true;
}

/** Get bucket_count value.
 * 
		  The number of valid entries in bucket array fields.
	  
 * @return bucket_count value
 */
uint32_t
MetricHistogramInterface::bucket_count() const
{
  return data->bucket_count;
}

/** Get maximum length of bucket_count value.
 * @return length of bucket_count value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MetricHistogramInterface::maxlenof_bucket_count() const
{
  return 1;
}

/** Set bucket_count value.
 * 
		  The number of valid entries in bucket array fields.
	  
 * @param new_bucket_count new bucket_count value
 */
void
MetricHistogramInterface::set_bucket_count(const uint32_t new_bucket_count)
{
  data->bucket_count = new_bucket_count;
  data_changed = true;
}

/** Get bucket_cumulative_count value.
 * 
		  The cumulative number of elements for the buckets.
	  
 * @return bucket_cumulative_count value
 */
uint64_t *
MetricHistogramInterface::bucket_cumulative_count() const
{
  return data->bucket_cumulative_count;
}

/** Get bucket_cumulative_count value at given index.
 * 
		  The cumulative number of elements for the buckets.
	  
 * @param index index of value
 * @return bucket_cumulative_count value
 * @exception Exception thrown if index is out of bounds
 */
uint64_t
MetricHistogramInterface::bucket_cumulative_count(unsigned int index) const
{
  if (index > 16) {
    throw Exception("Index value %u out of bounds (0..16)", index);
  }
  return data->bucket_cumulative_count[index];
}

/** Get maximum length of bucket_cumulative_count value.
 * @return length of bucket_cumulative_count value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MetricHistogramInterface::maxlenof_bucket_cumulative_count() const
{
  return 16;
}

/** Set bucket_cumulative_count value.
 * 
		  The cumulative number of elements for the buckets.
	  
 * @param new_bucket_cumulative_count new bucket_cumulative_count value
 */
void
MetricHistogramInterface::set_bucket_cumulative_count(const uint64_t * new_bucket_cumulative_count)
{
  memcpy(data->bucket_cumulative_count, new_bucket_cumulative_count, sizeof(uint64_t) * 16);
  data_changed = true;
}

/** Set bucket_cumulative_count value at given index.
 * 
		  The cumulative number of elements for the buckets.
	  
 * @param new_bucket_cumulative_count new bucket_cumulative_count value
 * @param index index for of the value
 */
void
MetricHistogramInterface::set_bucket_cumulative_count(unsigned int index, const uint64_t new_bucket_cumulative_count)
{
  if (index > 16) {
    throw Exception("Index value %u out of bounds (0..16)", index);
  }
  data->bucket_cumulative_count[index] = new_bucket_cumulative_count;
  data_changed = true;
}
/** Get bucket_upper_bound value.
 * 
		  The upper bound for the given bucket.
	  
 * @return bucket_upper_bound value
 */
double *
MetricHistogramInterface::bucket_upper_bound() const
{
  return data->bucket_upper_bound;
}

/** Get bucket_upper_bound value at given index.
 * 
		  The upper bound for the given bucket.
	  
 * @param index index of value
 * @return bucket_upper_bound value
 * @exception Exception thrown if index is out of bounds
 */
double
MetricHistogramInterface::bucket_upper_bound(unsigned int index) const
{
  if (index > 16) {
    throw Exception("Index value %u out of bounds (0..16)", index);
  }
  return data->bucket_upper_bound[index];
}

/** Get maximum length of bucket_upper_bound value.
 * @return length of bucket_upper_bound value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MetricHistogramInterface::maxlenof_bucket_upper_bound() const
{
  return 16;
}

/** Set bucket_upper_bound value.
 * 
		  The upper bound for the given bucket.
	  
 * @param new_bucket_upper_bound new bucket_upper_bound value
 */
void
MetricHistogramInterface::set_bucket_upper_bound(const double * new_bucket_upper_bound)
{
  memcpy(data->bucket_upper_bound, new_bucket_upper_bound, sizeof(double) * 16);
  data_changed = true;
}

/** Set bucket_upper_bound value at given index.
 * 
		  The upper bound for the given bucket.
	  
 * @param new_bucket_upper_bound new bucket_upper_bound value
 * @param index index for of the value
 */
void
MetricHistogramInterface::set_bucket_upper_bound(unsigned int index, const double new_bucket_upper_bound)
{
  if (index > 16) {
    throw Exception("Index value %u out of bounds (0..16)", index);
  }
  data->bucket_upper_bound[index] = new_bucket_upper_bound;
  data_changed = true;
}
/* =========== message create =========== */
Message *
MetricHistogramInterface::create_message(const char *type) const
{
  throw UnknownTypeException("The given type '%s' does not match any known "
                             "message type for this interface type.", type);
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
MetricHistogramInterface::copy_values(const Interface *other)
{
  const MetricHistogramInterface *oi = dynamic_cast<const MetricHistogramInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(MetricHistogramInterface_data_t));
}

const char *
MetricHistogramInterface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
MetricHistogramInterface::message_valid(const Message *message) const
{
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(MetricHistogramInterface)
/// @endcond


} // end namespace fawkes
