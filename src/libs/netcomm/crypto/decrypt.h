
/***************************************************************************
 *  decrypt.h - Message decryption routine
 *
 *  Created: Thu May 03 15:53:20 2007
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

#ifndef __NETCOMM_CRYPTO_DECRYPT_H_
#define __NETCOMM_CRYPTO_DECRYPT_H_

#include <core/exception.h>
#include <cstddef>

namespace fawkes {

class MessageDecryptionException : public Exception
{
 public:
  MessageDecryptionException(const char *msg);
};


class MessageDecryptor
{
 public:
  MessageDecryptor(const unsigned char *key, const unsigned char *iv);
  ~MessageDecryptor();

  void set_plain_buffer(void *buffer, size_t buffer_length);
  void set_crypt_buffer(void *buffer, size_t buffer_length);

  size_t decrypt();

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
