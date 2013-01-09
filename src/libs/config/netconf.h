
/***************************************************************************
 *  netconf.h - Fawkes remote configuration access via Fawkes net
 *
 *  Created: Sun Jan 07 15:01:50 2007
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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

#ifndef __CONFIG_NETCONF_H_
#define __CONFIG_NETCONF_H_

#include <config/config.h>
#include <netcomm/fawkes/client_handler.h>
#include <core/exception.h>

#include <map>
#include <list>
#include <string>
#include <stdint.h>

namespace fawkes {

class Mutex;
class InterruptibleBarrier;
class FawkesNetworkClient;
class MemoryConfiguration;

class CannotEnableMirroringException : public Exception
{
 public:
  CannotEnableMirroringException(const char *msg);
};

class NetworkConfiguration : public Configuration, public FawkesNetworkClientHandler
{
 public:
  NetworkConfiguration(FawkesNetworkClient *c, unsigned int mirror_timeout_sec = 15);
  virtual ~NetworkConfiguration();

  virtual void          copy(Configuration *copyconf);

  virtual void          add_change_handler(ConfigurationChangeHandler *h);
  virtual void          rem_change_handler(ConfigurationChangeHandler *h);

  virtual void          load(const char *file_path);

  virtual bool          exists(const char *path);
  virtual bool          is_float(const char *path);
  virtual bool          is_uint(const char *path);
  virtual bool          is_int(const char *path);
  virtual bool          is_bool(const char *path);
  virtual bool          is_string(const char *path);
  virtual bool          is_list(const char *path);

  virtual bool          is_default(const char *path);

  virtual float           get_float(const char *path);
  virtual unsigned int    get_uint(const char *path);
  virtual int             get_int(const char *path);
  virtual bool            get_bool(const char *path);
  virtual std::string     get_string(const char *path);
  virtual std::vector<float>         get_floats(const char *path);
  virtual std::vector<unsigned int>  get_uints(const char *path);
  virtual std::vector<int>           get_ints(const char *path);
  virtual std::vector<bool>          get_bools(const char *path);
  virtual std::vector<std::string>   get_strings(const char *path);
  virtual ValueIterator * get_value(const char *path);
  virtual std::string     get_comment(const char *path);
  virtual std::string     get_default_comment(const char *path);
  virtual std::string     get_type(const char *path);

  virtual void          set_float(const char *path, float f);
  virtual void          set_uint(const char *path, unsigned int uint);
  virtual void          set_int(const char *path, int i);
  virtual void          set_bool(const char *path, bool b);
  virtual void          set_string(const char *path, std::string &s);
  virtual void          set_string(const char *path, const char *s);
  virtual void          set_floats(const char *path, std::vector<float> &f);
  virtual void          set_uints(const char *path, std::vector<unsigned int> &uint);
  virtual void          set_ints(const char *path, std::vector<int> &i);
  virtual void          set_bools(const char *path, std::vector<bool> &b);
  virtual void          set_strings(const char *path, std::vector<std::string> &s);
  virtual void          set_strings(const char *path, std::vector<const char *> &s);
  virtual void          set_comment(const char *path, std::string &comment);
  virtual void          set_comment(const char *path, const char *comment);

  virtual void          erase(const char *path);

  virtual void          set_default_float(const char *path, float f);
  virtual void          set_default_uint(const char *path, unsigned int uint);
  virtual void          set_default_int(const char *path, int i);
  virtual void          set_default_bool(const char *path, bool b);
  virtual void          set_default_string(const char *path, std::string &s);
  virtual void          set_default_string(const char *path, const char *s);
  virtual void          set_default_comment(const char *path, std::string &comment);
  virtual void          set_default_comment(const char *path, const char *comment);

  virtual void          erase_default(const char *path);

  virtual void          deregistered(unsigned int id) throw();
  virtual void          inbound_received(FawkesNetworkMessage *msg,
					 unsigned int id) throw();
  virtual void          connection_died(unsigned int id) throw();
  virtual void          connection_established(unsigned int id) throw();

  virtual void          set_mirror_mode(bool mirror);

 class NetConfValueIterator : public Configuration::ValueIterator
  {
    friend class NetworkConfiguration;
   protected:
    NetConfValueIterator(Configuration::ValueIterator *i);
    NetConfValueIterator(FawkesNetworkMessage *m);
    NetConfValueIterator();
   public:
    virtual ~NetConfValueIterator();
    virtual bool          next();
    virtual bool          valid() const;

    virtual const char *  path() const;
    virtual const char *  type() const;

    virtual bool          is_float() const;
    virtual bool          is_uint() const;
    virtual bool          is_int() const;
    virtual bool          is_bool() const;
    virtual bool          is_string() const;
    virtual bool          is_list() const;
    virtual size_t        get_list_size() const;

    virtual bool          is_default() const;

    virtual float         get_float() const;
    virtual unsigned int  get_uint() const;
    virtual int           get_int() const;
    virtual bool          get_bool() const;
    virtual std::string   get_string() const;
    virtual std::vector<float>         get_floats() const;
    virtual std::vector<unsigned int>  get_uints() const;
    virtual std::vector<int>           get_ints() const;
    virtual std::vector<bool>          get_bools() const;
    virtual std::vector<std::string>   get_strings() const;
    virtual std::string   get_as_string() const;

    virtual std::string   get_comment() const;

   private:
    Configuration::ValueIterator *i;
    FawkesNetworkMessage  *msg;
    bool iterated_once;
    char *_path;
  };

  ValueIterator * iterator();
  ValueIterator * iterator_default();
  ValueIterator * iterator_hostspecific();
  ValueIterator * search(const char *path);

  void lock();
  bool try_lock();
  void unlock();

  virtual void try_dump();

 private:
  void send_get(const char *path, unsigned int msgid);

  void set_value_internal(unsigned int msg_type, const char *path, uint16_t num_values,
			  size_t data_size, void *data);

  void erase_internal(const char *path, bool is_default);


  FawkesNetworkClient  *c;
  FawkesNetworkMessage *msg;
  Mutex *mutex;
  InterruptibleBarrier *__mirror_init_barrier;

  bool __mirror_mode;
  bool __mirror_mode_before_connection_dead;
  unsigned int __mirror_timeout_sec;
  MemoryConfiguration *mirror_config;

  bool __connected;
};

} // end namespace fawkes

#endif
