
#include <fvutils/writers/fvraw.h>

#include <cstdlib>
#include <cstring>

using namespace firevision;

int
main(int argc, char **argv)
{
  unsigned char *buf = (unsigned char *)malloc(640 * 480);
  memset(buf, 0, 640*480);
  unsigned char *b = buf;

  for (unsigned int h = 0; h < 480; h += 2) {
    for (unsigned int w = 0; w < 640; w += 2) {
      *b++ = 255;
      ++b;
    }
    for (unsigned int w = 0; w < 640; w += 2) {
      ++b;
      *b++ = 255;
    }
  }

  FvRawWriter w("test.raw", 640, 480, MONO8, buf);
  w.write();

  return 0;
}
