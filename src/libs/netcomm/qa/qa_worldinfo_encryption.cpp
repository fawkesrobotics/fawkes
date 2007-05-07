
/***************************************************************************
 *  qa_worldinfo_encrypt.cpp - Fawkes QA WorldInfo encryption
 *
 *  Created: Fri May 04 13:38:50 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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

/// @cond QA

#include <netcomm/worldinfo/encrypt.h>
#include <netcomm/worldinfo/decrypt.h>

#include <cstddef>
#include <iostream>

using namespace std;

#define MAXLENGTH 1200

int
main(int argc, char **argv)
{
  // ArgumentParser *argp = new ArgumentParser(argc, argv, "rl");

  WorldInfoMessageEncryptor *e = new WorldInfoMessageEncryptor((const unsigned char *)"QAKEY",
							       (const unsigned char *)"QAIV123456");
  WorldInfoMessageDecryptor *d = new WorldInfoMessageDecryptor((const unsigned char *)"QAKEY",
							       (const unsigned char *)"QAIV123456");
  

  char *input = (char *)malloc(MAXLENGTH);
  char *output = (char *)malloc(MAXLENGTH);
  e->set_plain_buffer(input, MAXLENGTH);
  char *crypted = (char *)malloc(e->recommended_crypt_buffer_size());
  e->set_crypt_buffer(crypted, e->recommended_crypt_buffer_size());

  strncpy(input, "Test String 12345", MAXLENGTH);
  printf("Plain text: %s\n", input);

  e->set_plain_buffer(input, strlen(input));
  long unsigned int bytes = e->encrypt();

  printf("Encrypted to %lu bytes ", bytes);
  //for (size_t i = 0; i < bytes; i += 4) {
  //  printf("%x", crypted[i]);
  //}
  printf("\n");

  memset(output, 0, MAXLENGTH);
  d->set_crypt_buffer(crypted, bytes);
  d->set_plain_buffer(output, MAXLENGTH);
  bytes = d->decrypt();

  printf("Decrypted to %lu bytes: %s\n", bytes, output);

  free(input);
  free(output);

  delete e;
  delete d;
  //delete argp;
  return 0;
}

/// @endcond
