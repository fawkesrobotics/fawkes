
/***************************************************************************
 *  config.cpp - Fawkes configuration interface
 *
 *  Created: Mon Dec 18 14:54:23 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#include <config/config.h>


/** @class Configuration config/config.h
 * Interface for configuration handling.
 * We know that half of robotics is about parameter tuning. The Configuration
 * interface defines a unified way of storing parameters and other
 * configuration options no matter of how the database is implemented.
 * This is mainly done to allow for testing different solutions for ticket #10.
 *
 * @fn Configuration::~Configuration()
 * Virtual empty destructor.
 *
 * @fn void Configuration::add_change_handler(ConfigurationChangeHandler *h)
 * Add a configuration change handler.
 * The added handler is called whenever a value changes and the handler
 * desires to get notified for the given component.
 * @param h configuration change handler
 *
 * @fn void Configuration::rem_change_handler(ConfigurationChangeHandler *h)
 * Remove a configuration change handler.
 * The handler is removed from the change handler list and no longer called on
 * config changes.
 * @param h configuration change handler
 *
 * @fn void Configuration::load(const char *name, const char *defaults_name, const char *tag)
 * Load configuration.
 * Loads configuration data, or opens a file, depending on the implementation. After
 * this call access to all other methods shall be possible.
 * @param name name of the host-based configuration. If this does not exist it shall
 * be created from the default configuration. The name depends on the implementation and
 * could be a filename.
 * @param defaults_name name of the default database. As for the name this depends on
 * the actual implementation.
 * @param tag this optional parameter can denote a specific config version to load. This
 * will cause the host-specific database to be flushed and filled with the values for
 * the given tag. All values that did not exist for the tag are copied over from the
 * default database.
 * 
 * @fn void Configuration::tag(const char *tag)
 * Tag this configuration version.
 * This creates a new tagged version of the current config. The tagged config can be
 * accessed via load().
 * @param tag tag for this version
 *
 * @fn void Configuration::copy(Configuration *copyconf)
 * Copy all values from the given configuration.
 * All values from the given configuration are copied. Old values are not erased
 * so that the copied values will overwrite existing values, new values are
 * created, but values existent in current config but not in the copie config
 * will remain unchanged.
 * @param copyconf configuration to copy
 * 
 * @fn std::list<std::string> Configuration::tags()
 * List of tags.
 * @return list of tags
 * 
 * @fn bool Configuration::exists(const char *comp, const char *path)
 * Check if a given value exists.
 * @param comp component
 * @param path path to value
 * @return true if the value exists, false otherwise
 * 
 * @fn bool Configuration::is_float(const char *comp, const char *path)
 * Check if a value is of type float
 * @param comp component
 * @param path path to value
 * @return true if the value exists and is of type float
 * 
 * @fn bool Configuration::is_uint(const char *comp, const char *path)
 * Check if a value is of type unsigned int
 * @param comp component
 * @param path path to value
 * @return true if the value exists and is of type unsigned int
 * 
 * @fn bool Configuration::is_int(const char *comp, const char *path)
 * Check if a value is of type int
 * @param comp component
 * @param path path to value
 * @return true if the value exists and is of type int
 * 
 * @fn bool Configuration::is_bool(const char *comp, const char *path)
 * Check if a value is of type bool
 * @param comp component
 * @param path path to value
 * @return true if the value exists and is of type bool
 * 
 * @fn bool Configuration::is_string(const char *comp, const char *path)
 * Check if a value is of type string
 * @param comp component
 * @param path path to value
 * @return true if the value exists and is of type string
 * 
 * @fn float Configuration::get_float(const char *comp, const char *path)
 * Get value from configuration which is of type float
 * @param comp component
 * @param path path to value
 * @return value
 * 
 * @fn unsigned int Configuration::get_uint(const char *comp, const char *path)
 * Get value from configuration which is of type unsigned int
 * @param comp component
 * @param path path to value
 * @return value
 * 
 * @fn int Configuration::get_int(const char *comp, const char *path)
 * Get value from configuration which is of type int
 * @param comp component
 * @param path path to value
 * @return value
 * 
 * @fn bool Configuration::get_bool(const char *comp, const char *path)
 * Get value from configuration which is of type bool
 * @param comp component
 * @param path path to value
 * @return value
 * 
 * @fn std::string Configuration::get_string(const char *comp, const char *path)
 * Get value from configuration which is of type string
 * @param comp component
 * @param path path to value
 *
 * @fn Configuration::ValueIterator * Configuration::get_value(const char *comp, const char *path)
 * Get value from configuration.
 * @param comp component
 * @param path path to value
 * @return value iterator for just this one value, maybe invalid if value does not
 * exists.
 *
 * 
 * @fn void Configuration::set_float(const char *comp, const char *path, float f)
 * Set new value in configuration of type float
 * @param comp component
 * @param path path to value
 * @param f new float value
 * 
 * @fn void Configuration::set_uint(const char *comp, const char *path, unsigned int uint)
 * Set new value in configuration of type unsigned int
 * @param comp component
 * @param path path to value
 * @param uint new unsigned int value
 * 
 * @fn void Configuration::set_int(const char *comp, const char *path, int i)
 * Set new value in configuration of type int
 * @param comp component
 * @param path path to value
 * @param i new int value
 * 
 * @fn void Configuration::set_bool(const char *comp, const char *path, bool b)
 * Set new value in configuration of type bool
 * @param comp component
 * @param path path to value
 * @param b new bool value
 * 
 * @fn void Configuration::set_string(const char *comp, const char *path, std::string s)
 * Set new value in configuration of type string
 * @param comp component
 * @param path path to value
 * @param s new string value
 *
 * @fn void Configuration::set_string(const char *comp, const char *path, const char *s)
 * Set new value in configuration of type string. Works like the aforementioned method.
 * Just takes an good ol' char array instead of a std::string.
 * @param comp component
 * @param path path to value
 * @param s new string value
 *
 * @fn void Configuration::erase(const char *comp, const char *path)
 * Erase the given value from the configuration. It is not an error if the value does
 * not exists before deletion.
 * @param comp component
 * @param path path to value
 *
 * @fn void Configuration::set_default_float(const char *comp, const char *path, float f)
 * Set new default value in configuration of type float
 * @param comp component
 * @param path path to value
 * @param f new float value
 * 
 * @fn void Configuration::set_default_uint(const char *comp, const char *path, unsigned int uint)
 * Set new default value in configuration of type unsigned int
 * @param comp component
 * @param path path to value
 * @param uint new unsigned int value
 * 
 * @fn void Configuration::set_default_int(const char *comp, const char *path, int i)
 * Set new default value in configuration of type int
 * @param comp component
 * @param path path to value
 * @param i new int value
 * 
 * @fn void Configuration::set_default_bool(const char *comp, const char *path, bool b)
 * Set new default value in configuration of type bool
 * @param comp component
 * @param path path to value
 * @param b new bool value
 * 
 * @fn void Configuration::set_default_string(const char *comp, const char *path, std::string s)
 * Set new default value in configuration of type string
 * @param comp component
 * @param path path to value
 * @param s new string value
 *
 * @fn void Configuration::set_default_string(const char *comp, const char *path, const char *s)
 * Set new default value in configuration of type string. Works like the aforementioned method.
 * Just takes an good ol' char array instead of a std::string.
 * @param comp component
 * @param path path to value
 * @param s new string value
 *
 * @fn void Configuration::erase_default(const char *comp, const char *path)
 * Erase the given default value from the configuration. It is not an error if the value does
 * not exists before deletion.
 * @param comp component
 * @param path path to value
 *
 * @fn Configuration::ValueIterator * Configuration::iterator()
 * Iterator for all values.
 * Returns an iterator that can be used to iterate over all values in the current
 * configuration.
 * @return iterator over all values
 *
 * @fn Configuration::ValueIterator * Configuration::search(const char *component, const char *path)
 * Iterator with search results.
 * Returns an iterator that can be used to iterate over the search results. All values
 * whose component and path start with the given strings are returned.
 * A call like
 * @code
 *   config->search("", "");
 * @endcode
 * is effectively the same as a call to iterator().
 * @param component start of component
 * @param path start of path
 * @return iterator to search results
 *
 * @fn void Configuration::lock()
 * Lock the config.
 * No further changes or queries can be executed on the configuration and will block until
 * the config is unlocked.
 *
 * @fn bool Configuration::tryLock()
 * Try to lock the config.
 * @see Configuration::lock()
 * @return true, if the lock has been aquired, false otherwise
 *
 * @fn void Configuration::unlock()
 * Unlock the config.
 * Modifications and queries are possible again.
 *
 */

/** @class ConfigurationException config/config.h
 * Generic configuration exception.
 * Thrown if there is no other matching exception.
 */


/** Constructor.
 * @param msg message
 */
ConfigurationException::ConfigurationException(const char *msg)
  : Exception(msg)
{
}


/** Constructor.
 * @param prefix Put as "prefix: " before the message, can be used to have a prefix
 * and put an error message from another API into msg.
 * @param msg message
 */
ConfigurationException::ConfigurationException(const char *prefix, const char *msg)
  : Exception()
{
  append("%s: %s", prefix, msg);
}


/** @class ConfigEntryNotFoundException config/config.h
 * Thrown if a config entry could not be found.
 */


/** Constructor.
 * @param component component of value
 * @param path path of value
 */
ConfigEntryNotFoundException::ConfigEntryNotFoundException(const char *component,
							   const char *path)
  : Exception()
{
  append("Config value for '%s::%s' not found", component, path);
}


/** @class ConfigTypeMismatchException config/config.h
 * Thrown if there a type problem was detected for example if you tried
 * to query a float with get_int().
 */

/** Constructor.
 * @param component component of value
 * @param path path of value
 * @param actual actual type
 * @param requested requested type
 */
ConfigTypeMismatchException::ConfigTypeMismatchException(const char *component,
							 const char *path,
							 const char *actual,
							 const char *requested)
  : Exception()
{
  append("Config value for '%s::%s' is not of type '%s', but of type '%s'",
	 component, path, requested, actual);
}

/** @class CouldNotOpenConfigException config/config.h
 * Thrown if config could not be opened.
 * This is most likely to happen during the constructor or load().
 */

/** Constructor.
 * @param msg cause or symptom of failure
 */
CouldNotOpenConfigException::CouldNotOpenConfigException(const char *msg)
  : Exception(msg)
{
}



/** @class Configuration::ValueIterator config/config.h
 * Iterator interface to iterate over config values. This does not implement a
 * classic iterator interface with begin and end nodes but rather mimics a more
 * Java-like interface where you iterate over the entries in a while loop until
 * you covered all entries (much like a queue).
 * If you implement this for your own configuration system you should not make
 * the constructor publically accessible.
 *
 * @fn Configuration::ValueIterator::~ValueIterator()
 * Virtual emptry destructor.
 *
 * @fn bool Configuration::ValueIterator::next()
 * Check if there is another element and advance to this if possible.
 * This advances to the next element, if there is one.
 * @return true, if another element has been reached, false otherwise
 *
 * @fn bool Configuration::ValueIterator::valid()
 * Check if the current element is valid.
 * This is much like the classic end element for iterators. If the iterator is
 * invalid there all subsequent calls to next() shall fail.
 * @return true, if the iterator is still valid, false otherwise
 *
 * @fn const char * Configuration::ValueIterator::component()
 * Component of value.
 * @return component of value.
 *
 * @fn const char * Configuration::ValueIterator::path()
 * Path of value.
 * @return path of value
 *
 * @fn const char * Configuration::ValueIterator::type()
 * Type of value.
 * @return string representation of value type.
 *
 * @fn bool Configuration::ValueIterator::is_float()
 * Check if current value is a float.
 * @return true, if value is a float, false otherwise
 *
 * @fn bool Configuration::ValueIterator::is_uint()
 * Check if current value is a unsigned int.
 * @return true, if value is a unsigned int, false otherwise
 *
 * @fn bool Configuration::ValueIterator::is_int()
 * Check if current value is a int.
 * @return true, if value is a int, false otherwise
 *
 * @fn bool Configuration::ValueIterator::is_bool()
 * Check if current value is a bool.
 * @return true, if value is a bool, false otherwise
 *
 * @fn bool Configuration::ValueIterator::is_string()
 * Check if current value is a string.
 * @return true, if value is a string, false otherwise
 *
 * @fn float Configuration::ValueIterator::get_float()
 * Get float value.
 * @return value
 *
 * @fn unsigned int Configuration::ValueIterator::get_uint()
 * Get unsigned int value.
 * @return value
 *
 * @fn int Configuration::ValueIterator::get_int()
 * Get int value.
 * @return value
 *
 * @fn bool Configuration::ValueIterator::get_bool()
 * Get bool value.
 * @return value
 *
 * @fn std::string Configuration::ValueIterator::get_string()
 * Get string value.
 * @return value
 *
 */
