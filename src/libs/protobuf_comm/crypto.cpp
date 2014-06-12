
/***************************************************************************
 *  crypto.cpp - Protobuf stream protocol - crypto utils
 *
 *  Created: Tue Mar 11 21:14:58 2014
 *  Copyright  2014  Tim Niemueller [www.niemueller.de]
 ****************************************************************************/

/*  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in
 *   the documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the authors nor the names of its contributors
 *   may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <protobuf_comm/crypto.h>
#include <protobuf_comm/frame_header.h>

#include <stdexcept>
#ifdef HAVE_LIBCRYPTO
#  include <openssl/evp.h>
#  include <openssl/rand.h>
#  include <openssl/sha.h>
#  include <cstring>
#endif

namespace protobuf_comm {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class BufferEncryptor <protobuf_comm/crypto.h>
 * Encrypt buffers using AES128 in ECB mode.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param key encryption key, can be any string, will be processed to meet
 * the cipher's requirements.
 * @param cipher_name Cipher combination to use, currently supported are
 * aes-128-ecb, aes-128-cbc, aes-256-ecb, and aes-256-cbc
 */
BufferEncryptor::BufferEncryptor(const std::string &key, std::string cipher_name)
{
  cipher_ = cipher_by_name(cipher_name.c_str());
  cipher_id_ = cipher_name_to_id(cipher_name.c_str());

  const size_t key_size = EVP_CIPHER_key_length(cipher_);
  const size_t iv_size = EVP_CIPHER_iv_length(cipher_);
  key_ = (unsigned char *)malloc(key_size);
  unsigned char iv[iv_size];
  if( ! EVP_BytesToKey(cipher_, EVP_sha256(), NULL,
		       (const unsigned char *)key.c_str(), key.size(), 8, key_, iv))
  {
    throw std::runtime_error("Failed to generate key");
  }

  if (!RAND_bytes((unsigned char *)&iv_, sizeof(iv_))) {
    throw std::runtime_error("Failed to generate IV");
  }
}


/** Destructor. */
BufferEncryptor::~BufferEncryptor()
{
  free(key_);
}


/** Encrypt a buffer.
 * Uses the cipher set in the constructor.
 * @param plain plain text data
 * @param enc upon return contains encrypted buffer
 */
void
BufferEncryptor::encrypt(const std::string &plain, std::string &enc)
{
#ifdef HAVE_LIBCRYPTO
  const EVP_CIPHER *evp_cipher = cipher_by_id(cipher_id_);

  const size_t iv_size = EVP_CIPHER_iv_length(evp_cipher);
  unsigned char iv_hash[SHA256_DIGEST_LENGTH];

  unsigned char *enc_m = (unsigned char *)enc.c_str();

  if (iv_size > 0) {
    iv_ += 1;
    
    if (! SHA256((unsigned char *)&iv_, sizeof(iv_), iv_hash)) {
      throw std::runtime_error("Failed to generate IV");
    }
    enc.replace(0, iv_size, (char *)iv_hash, iv_size);
    enc_m += iv_size;
  }

  EVP_CIPHER_CTX ctx;
  if ( ! EVP_EncryptInit(&ctx, evp_cipher, key_, iv_hash))
  {
    throw std::runtime_error("Could not initialize cipher context");
  }

  int outl = enc.size() - iv_size;
  if ( ! EVP_EncryptUpdate(&ctx, enc_m, &outl,
			   (unsigned char *)plain.c_str(), plain.size()) )
  {
    throw std::runtime_error("EncryptUpdate failed");
  }

  int plen = 0;
  if ( ! EVP_EncryptFinal_ex(&ctx, enc_m + outl, &plen) ) {
    throw std::runtime_error("EncryptFinal failed");
  }
  outl += plen;
 
  enc.resize(outl + iv_size);
#else
  throw std::runtime_error("Encryption support not available");
#endif

}


/** Get required size for an encrypted buffer of the given plain text length.
 * @param plain_length length of the plain text buffer to encrypt
 * @return length of encrypted buffer required
 */
size_t
BufferEncryptor::encrypted_buffer_size(size_t plain_length)
{
#ifdef HAVE_LIBCRYPTO
  const EVP_CIPHER *evp_cipher = cipher_by_id(cipher_id_);

  const size_t iv_size = EVP_CIPHER_iv_length(evp_cipher);
  size_t block_size    = EVP_CIPHER_block_size(evp_cipher);

  return (((plain_length / block_size) + 1) * block_size) + iv_size;
#else
  throw std::runtime_error("Encryption not supported");
#endif
}


/** @class BufferDecryptor <protobuf_comm/crypto.h>
 * Decrypt buffers encrypted with BufferEncryptor.
 * @author Tim Niemueller
 */


/** Constructor.
 * @param key encryption key, can be any string, will be processed to meet
 * AES128 requirements.
 */
BufferDecryptor::BufferDecryptor(const std::string &key)
  : key_(key)
{
}


/** Destructor. */
BufferDecryptor::~BufferDecryptor()
{
}


void
BufferDecryptor::generate_key(int cipher)
{
  const EVP_CIPHER *evp_cipher = cipher_by_id(cipher);

  const size_t key_size = EVP_CIPHER_key_length(evp_cipher);
  const size_t iv_size = EVP_CIPHER_iv_length(evp_cipher);
  unsigned char *key = (unsigned char *)malloc(key_size);
  unsigned char iv[iv_size];
  if( ! EVP_BytesToKey(evp_cipher, EVP_sha256(), NULL,
		       (const unsigned char *)key_.c_str(), key_.size(), 8, key, iv))
  {
    free(key);
    throw std::runtime_error("Failed to generate key");
  }

  std::string ks((const char *)key, key_size);
  free(key);

  keys_[cipher] = ks;
}


/** Decrypt a buffer.
 * @param cipher cipher ID
 * @param enc encrypted buffer
 * @param enc_size number of bytes in @p enc
 * @param plain on return contains plain text data
 * @param plain_size size in bytes of @p plain
 * @return number of bytes that were in the encrypted buffer (this can be shorter if the data
 * did not exactly fit the AES block size.
 */
size_t
BufferDecryptor::decrypt(int cipher, const void *enc, size_t enc_size, void *plain, size_t plain_size)
{
#ifdef HAVE_LIBCRYPTO
  if (keys_.find(cipher) == keys_.end()) {
    generate_key(cipher);
  }

  const EVP_CIPHER *evp_cipher = cipher_by_id(cipher);

  const size_t iv_size = EVP_CIPHER_iv_length(evp_cipher);
  const unsigned char *iv = (const unsigned char *)enc;
  unsigned char *enc_m = (unsigned char *)enc + iv_size;
  enc_size -= iv_size;

  EVP_CIPHER_CTX ctx;
  if ( ! EVP_DecryptInit(&ctx, evp_cipher, (const unsigned char *)keys_[cipher].c_str(), iv))
  {
    throw std::runtime_error("Could not initialize cipher context");
  }

  int outl = plain_size;
  if ( ! EVP_DecryptUpdate(&ctx,
			   (unsigned char *)plain, &outl, enc_m, enc_size))
  {
    throw std::runtime_error("DecryptUpdate failed");
  }

  int plen = 0;
  if ( ! EVP_DecryptFinal(&ctx, (unsigned char *)plain + outl, &plen) ) {
    throw std::runtime_error("DecryptFinal failed");
  }
  outl += plen;

  return outl;
#else
  throw std::runtime_error("Decryption support not available");
#endif
}


/** Get cipher name for PB_ENCRYPTION_* constants.
 * @param cipher cipher ID
 * @return string representing the cipher
 */
const char *
cipher_name_by_id(int cipher)
{
  switch (cipher) {
  case PB_ENCRYPTION_AES_128_ECB:
    return SN_aes_128_ecb;
  case PB_ENCRYPTION_AES_128_CBC:
    return SN_aes_128_cbc;

  case PB_ENCRYPTION_AES_256_ECB:
    return SN_aes_256_ecb;
  case PB_ENCRYPTION_AES_256_CBC:
    return SN_aes_256_cbc;

  default:
    throw std::runtime_error("Unknown cipher type");
  }
}


/** Get cipher for PB_ENCRYPTION_* constants.
 * @param cipher cipher ID
 * @return cipher engine
 */
const EVP_CIPHER *
cipher_by_id(int cipher)
{
  switch (cipher) {
  case PB_ENCRYPTION_AES_128_ECB:
    return EVP_aes_128_ecb();
  case PB_ENCRYPTION_AES_128_CBC:
    return EVP_aes_128_cbc();

  case PB_ENCRYPTION_AES_256_ECB:
    return EVP_aes_256_ecb();
  case PB_ENCRYPTION_AES_256_CBC:
    return EVP_aes_256_cbc();

  default:
    throw std::runtime_error("Unknown cipher type");
  }
}

/** Get cipher by name constants.
 * @param cipher cipher name
 * @return cipher engine
 */
int
cipher_name_to_id(const char *cipher)
{
  if (strcmp(cipher, LN_aes_128_ecb) == 0) {
    return PB_ENCRYPTION_AES_128_ECB;
  } else if (strcmp(cipher, LN_aes_128_cbc) == 0) {
    return PB_ENCRYPTION_AES_128_CBC;
  } else if (strcmp(cipher, LN_aes_256_ecb) == 0) {
    return PB_ENCRYPTION_AES_256_ECB;
  } else if (strcmp(cipher, LN_aes_256_cbc) == 0) {
    return PB_ENCRYPTION_AES_256_CBC;
  } else {
    throw std::runtime_error("Unknown cipher type");
  }
}


/** Get cipher by name constants.
 * @param cipher cipher name
 * @return cipher engine
 */
const EVP_CIPHER *
cipher_by_name(const char *cipher)
{
  if (strcmp(cipher, LN_aes_128_ecb) == 0) {
    return EVP_aes_128_ecb();
  } else if (strcmp(cipher, LN_aes_128_cbc) == 0) {
    return EVP_aes_128_cbc();
  } else if (strcmp(cipher, LN_aes_256_ecb) == 0) {
    return EVP_aes_256_ecb();
  } else if (strcmp(cipher, LN_aes_256_cbc) == 0) {
    return EVP_aes_256_cbc();
  } else {
    throw std::runtime_error("Unknown cipher type");
  }
}


} // end namespace fawkes
