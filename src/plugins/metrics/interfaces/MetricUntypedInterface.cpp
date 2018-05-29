
/***************************************************************************
 *  MetricUntypedInterface.cpp - Fawkes BlackBoard Interface - MetricUntypedInterface
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

#include <interfaces/MetricUntypedInterface.h>

#include <core/exceptions/software.h>

#include <map>
#include <string>
#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class MetricUntypedInterface <interfaces/MetricUntypedInterface.h>
 * MetricUntypedInterface Fawkes BlackBoard Interface.
 * 
		  This is an untyped metric of some sort. Use the specific types
		  if possible and resort to the untyped metric if that does not
		  fit.
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
MetricUntypedInterface::MetricUntypedInterface() : Interface()
{
  data_size = sizeof(MetricUntypedInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (MetricUntypedInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_STRING, "labels", 512, data->labels);
  add_fieldinfo(IFT_DOUBLE, "value", 1, &data->value);
  unsigned char tmp_hash[] = {0x1e, 0xc7, 0x29, 0x51, 0xef, 0x61, 0xda, 0x3f, 0x88, 0x13, 0x9f, 0x70, 0xb7, 0x34, 0x46, 0x15};
  set_hash(tmp_hash);
}

/** Destructor */
MetricUntypedInterface::~MetricUntypedInterface()
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
MetricUntypedInterface::labels() const
{
  return data->labels;
}

/** Get maximum length of labels value.
 * @return length of labels value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MetricUntypedInterface::maxlenof_labels() const
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
MetricUntypedInterface::set_labels(const char * new_labels)
{
  strncpy(data->labels, new_labels, sizeof(data->labels));
  data_changed = true;
}

/** Get value value.
 * 
	    The counter value.
    
 * @return value value
 */
double
MetricUntypedInterface::value() const
{
  return data->value;
}

/** Get maximum length of value value.
 * @return length of value value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MetricUntypedInterface::maxlenof_value() const
{
  return 1;
}

/** Set value value.
 * 
	    The counter value.
    
 * @param new_value new value value
 */
void
MetricUntypedInterface::set_value(const double new_value)
{
  data->value = new_value;
  data_changed = true;
}

/* =========== message create =========== */
Message *
MetricUntypedInterface::create_message(const char *type) const
{
  throw UnknownTypeException("The given type '%s' does not match any known "
                             "message type for this interface type.", type);
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
MetricUntypedInterface::copy_values(const Interface *other)
{
  const MetricUntypedInterface *oi = dynamic_cast<const MetricUntypedInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(MetricUntypedInterface_data_t));
}

const char *
MetricUntypedInterface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
MetricUntypedInterface::message_valid(const Message *message) const
{
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(MetricUntypedInterface)
/// @endcond


} // end namespace fawkes
