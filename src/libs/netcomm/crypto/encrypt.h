
/***************************************************************************
 *  encrypt.h - Message encryption routine
 *
 *  Created: Thu May 03 15:01:13 2007
 *  Copyright  2006-2014  Tim Niemueller [www.niemueller.de]
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

#ifndef __NETCOMM_CRYPTO_ENCRYPT_H_
#define __NETCOMM_CRYPTO_ENCRYPT_H_

#include <core/exception.h>
#include <cstddef>

namespace fawkes {

class MessageEncryptionException : public Exception
{
 public:
  MessageEncryptionException(const char *msg);
};

class MessageEncryptor
{
 public:
  MessageEncryptor(const unsigned char *key, const unsigned char *iv);
  ~MessageEncryptor();

  void set_plain_buffer(void *buffer, size_t buffer_length);
  void set_crypt_buffer(void *buffer, size_t buffer_length);

  size_t recommended_crypt_buffer_size();

  size_t encrypt();

 private:
  void    *plain_buffer;
  size_t   plain_buffer_length;
  void    *crypt_buffer;
  size_t   crypt_buffer_length;

  const unsigned char *key;
  const unsigned char *iv;
};

} // end namespace fawkes

#endif
