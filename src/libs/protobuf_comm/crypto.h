
/***************************************************************************
 *  crypto.h - Protobuf stream protocol - crypto utils
 *
 *  Created: Tue Mar 11 21:12:35 2014
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

#ifndef __PROTOBUF_COMM_CRYPTO_H_
#define __PROTOBUF_COMM_CRYPTO_H_

#include <string>
#include <map>

#ifdef HAVE_LIBCRYPTO
#  include <openssl/ossl_typ.h>
#endif

namespace protobuf_comm {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class BufferEncryptor {
 public:
  BufferEncryptor(const std::string &key, std::string cipher_name = "AES-128-ECB");
  ~BufferEncryptor();

  void encrypt(const std::string &plain, std::string &enc);

  /** Get cipher ID.
   * @return cipher ID */
  int cipher_id() const
  { return cipher_id_; }

  size_t encrypted_buffer_size(size_t plain_length);

 private:
  unsigned char *key_;
  long long unsigned int iv_;

  const EVP_CIPHER *cipher_;

  int cipher_id_;
};


class BufferDecryptor {
 public:
  BufferDecryptor(const std::string &key);
  ~BufferDecryptor();

  size_t decrypt(int cipher, const void *enc, size_t enc_size, void *plain, size_t plain_size);

 private:
  void generate_key(int cipher);

 private:
  std::string key_;
  std::map<int, std::string> keys_;
};

const char * cipher_name_by_id(int cipher);
int          cipher_name_to_id(const char *cipher);

#ifdef HAVE_LIBCRYPTO
const EVP_CIPHER * cipher_by_id(int cipher);
const EVP_CIPHER * cipher_by_name(const char *cipher);
#endif

} // end namespace fawkes

#endif
