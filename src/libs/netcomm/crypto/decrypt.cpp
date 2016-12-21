
/***************************************************************************
 *  decrypt.cpp - Message decryption routine
 *
 *  Created: Thu May 03 15:54:24 2007
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

#include <core/exceptions/software.h>
#include <netcomm/crypto/decrypt.h>
#ifdef HAVE_LIBCRYPTO
#  include <openssl/evp.h>
#else
#  include <cstring>
#endif

namespace fawkes {

/** @class MessageDecryptionException <netcomm/crypto/decrypt.h>
 * Message decryption failed.
 * This exception shall be thrown if there was a problem decrypting a
 * world info message.
 * @ingroup NetComm
 */

/** Constructor.
 * @param msg message
 */
MessageDecryptionException::MessageDecryptionException(const char *msg)
  : Exception(msg)
{
}


/** @class MessageDecryptor <netcomm/crypto/decrypt.h>
 * Message decryptor.
 * This class is used to decrypt world info message after they have been
 * received.
 *
 * This is the opposite part of MessageEncryptor.
 *
 * This implementation uses OpenSSL for the AES encryption (in fact it uses the
 * accompanying libcrypto that comes with OpenSSL, not libopenssl itself). It is
 * almost everywhere available and easy to use.
 * 
 * @see MessageEncryptor
 * @ingroup NetComm
 * @author Tim Niemueller
 */


/** Constructor.
 * @param key encryption key
 * @param iv initialisation vector
 */
MessageDecryptor::MessageDecryptor(const unsigned char *key, const unsigned char *iv)
{
  plain_buffer = NULL;
  plain_buffer_length = 0;
  crypt_buffer = NULL;
  crypt_buffer_length = 0;

  this->key = key;
  this->iv  = iv;
}


/** Empty destructor. */
MessageDecryptor::~MessageDecryptor()
{
}


/** Set plain buffer.
 * This is the destination buffer to which the decrypted plain text is written.
 * @param buffer plain text buffer
 * @param buffer_length plain text buffer length
 */
void
MessageDecryptor::set_plain_buffer(void *buffer, size_t buffer_length)
{
  plain_buffer        = buffer;
  plain_buffer_length = buffer_length;
}


/** Set crypted buffer.
 * This is the source buffer which is decrypted.
 * @param buffer crypted text buffer
 * @param buffer_length crypted text buffer length
 */
void
MessageDecryptor::set_crypt_buffer(void *buffer, size_t buffer_length)
{
  crypt_buffer        = buffer;
  crypt_buffer_length = buffer_length;
}


/** Decrypt.
 * Do the decryption.
 * @return size of the plain text message.
 */
size_t
MessageDecryptor::decrypt()
{
  if ( (plain_buffer == NULL) || (plain_buffer_length == 0) ||
       (crypt_buffer == NULL) || (crypt_buffer_length == 0) ) {
    throw MissingParameterException("Buffer(s) not set for decryption");
  }

#ifdef HAVE_LIBCRYPTO
  EVP_CIPHER_CTX *ctx = EVP_CIPHER_CTX_new();
  if ( ! EVP_DecryptInit(ctx, EVP_aes_128_ecb(), key, iv) ) {
    EVP_CIPHER_CTX_free(ctx);
    throw MessageDecryptionException("Could not initialize cipher context");
  }

  int outl = plain_buffer_length;
  if ( ! EVP_DecryptUpdate(ctx,
			   (unsigned char *)plain_buffer, &outl,
			   (unsigned char *)crypt_buffer, crypt_buffer_length) ) {
    EVP_CIPHER_CTX_free(ctx);
    throw MessageDecryptionException("DecryptUpdate failed");
  }

  int plen = 0;
  if ( ! EVP_DecryptFinal(ctx, (unsigned char *)plain_buffer + outl, &plen) ) {
    EVP_CIPHER_CTX_free(ctx);
    throw MessageDecryptionException("DecryptFinal failed");
  }
  outl += plen;

  EVP_CIPHER_CTX_free(ctx);
  return outl;
#else
  // Plain-text copy-through for debugging.
  //memcpy(plain_buffer, crypt_buffer, crypt_buffer_length);
  //return crypt_buffer_length;
  throw Exception("Decryption support not available");
#endif
}

} // end namespace fawkes
