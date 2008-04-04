
/***************************************************************************
 *  encrypt.cpp - WorldInfo encryptio routine
 *
 *  Created: Thu May 03 15:21:00 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You can read the full text in the LICENSE file in the doc directory. 
 */

#include <core/exceptions/software.h>
#include <netcomm/worldinfo/encrypt.h>
#include <openssl/evp.h>

/** @class MessageEncryptionException <netcomm/worldinfo/encrypt.h>
 * Message encryption failed.
 * This exception shall be thrown if there was a problem encrypting a
 * world info message.
 * @ingroup NetComm
 */

/** Constructor.
 * @param msg message
 */
MessageEncryptionException::MessageEncryptionException(const char *msg)
  : Exception(msg)
{
}


/** @class WorldInfoMessageEncryptor <netcomm/worldinfo/encrypt.h>
 * WorldInfo message encryptor.
 * This class is used to encrypt world info message before they are sent
 * over the network.
 *
 * The used encryption is AES (128 bit) with a supplied key and initialisation
 * vector that both sides have to agree on.
 * The encryption is used in the less safe Electronic Code Book (ECB) mode. It
 * is prefered over Cipher Block Chaining (CBC) mode since we expect a very
 * unreliable transport medium (wifi in a totally crowded and signal-hostile
 * environment) where we could have severe packet loss. In CBC mode if you loose
 * a single packet you can not only not decrypt this packet that you didn't get,
 * but also not the directly following packages. In this case it can already
 * cause severe problems if about half of the packes are lost.
 *
 * We are merely interested in some kind of child-proof blinds that is just used
 * to make cheating too much work to be interesting. We actually don't care if
 * someone can decrypt our traffic with enough time, we just don't want other
 * teams to be able to decrypt our traffic during the game. Otherwise teams
 * could cheat and just read the network messages to know where the opponents
 * are instead of really detecting them using sensors.
 *
 * This implementation uses OpenSSL for the AES encryption (in fact it uses the
 * accompanying libcrypto that comes with OpenSSL, not libopenssl itself). It is
 * almost everywhere available and easy to use.
 * 
 * @ingroup NetComm
 * @author Tim Niemueller
 */


/** Constructor.
 * @param key encryption key
 * @param iv initialisation vector
 */
WorldInfoMessageEncryptor::WorldInfoMessageEncryptor(const unsigned char *key, const unsigned char *iv)
{
  plain_buffer = NULL;
  plain_buffer_length = 0;
  crypt_buffer = NULL;
  crypt_buffer_length = 0;

  this->key = key;
  this->iv  = iv;
}


/** Empty destructor. */
WorldInfoMessageEncryptor::~WorldInfoMessageEncryptor()
{
}




/** Set plain buffer.
 * This set the source buffer that is encrypted.
 * @param buffer plain buffer
 * @param buffer_length plain buffer length
 */
void
WorldInfoMessageEncryptor::set_plain_buffer(void *buffer, size_t buffer_length)
{
  plain_buffer        = buffer;
  plain_buffer_length = buffer_length;
}


/** Get recommended crypted buffer size.
 * The cryto text is in most cases longer than the plain text. This is because
 * we use a block cipher. This block cipher encrypts block of certain sizes (in case
 * of AES128 a block has a size of 16 bytes). If our data does not align to this block
 * size padding at the end is required to fill up the last block to the requested
 * size. Since this padding depends on the used cipher this convenience method
 * is provided to get the recommended minimum size depending on the plain text
 * buffer (that you have to set before you call this method.
 * @return recommended minimum size of the crypted buffer
 * @exception MissingParameterException thrown, if set_plain_buffer() has not
 * been called or if the supplied buffer had zero size.
 */
size_t
WorldInfoMessageEncryptor::recommended_crypt_buffer_size()
{
  if ( plain_buffer_length == 0 ) {
    throw MissingParameterException("plain buffer must be set and plain buffer size > 0");
  }

  EVP_CIPHER_CTX ctx;
  EVP_EncryptInit(&ctx, EVP_aes_128_ecb(), key, iv);
  size_t rv = plain_buffer_length + EVP_CIPHER_CTX_block_size(&ctx);
  EVP_CIPHER_CTX_cleanup(&ctx);

  return rv;
}


/** Set crypted buffer.
 * This set the destination buffer to which the encrypted message is written.
 * @param buffer crypted buffer
 * @param buffer_length crypted buffer length
 */
void
WorldInfoMessageEncryptor::set_crypt_buffer(void *buffer, size_t buffer_length)
{
  crypt_buffer        = buffer;
  crypt_buffer_length = buffer_length;
}


/** Encrypt.
 * Do the encryption.
 * @return size of the crypted message in bytes
 */
size_t
WorldInfoMessageEncryptor::encrypt()
{
  if ( (plain_buffer == NULL) || (plain_buffer_length == 0) ||
       (crypt_buffer == NULL) || (crypt_buffer_length == 0) ) {
    throw MissingParameterException("Buffer(s) not set for encryption");
  }

  EVP_CIPHER_CTX ctx;
  if ( ! EVP_EncryptInit(&ctx, EVP_aes_128_ecb(), key, iv) ) {
    throw MessageEncryptionException("Could not initialize cipher context");
  }


  int outl = crypt_buffer_length;
  if ( ! EVP_EncryptUpdate(&ctx,
			   (unsigned char *)crypt_buffer, &outl,
			   (unsigned char *)plain_buffer, plain_buffer_length) ) {
    throw MessageEncryptionException("EncryptUpdate failed");
  }

  int plen = 0;
  if ( ! EVP_EncryptFinal_ex(&ctx, (unsigned char *)crypt_buffer + outl, &plen) ) {
    throw MessageEncryptionException("EncryptFinal failed");
  }
  outl += plen;

  return outl;
}
