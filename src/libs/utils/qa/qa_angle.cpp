
#include <utils/math/angle.h>

#include <stdio.h>

int
main(int argc, char **argv)
{
  float f = -2 * M_PI;
  float fnm = normalize_mirror_rad(f);
  float expd = 0;
  printf("f=%f   normalize_mirror_rad(f)=%f   expected=%f\n", f, fnm, expd);

  f = 2 * M_PI;
  fnm = normalize_mirror_rad(f);
  expd = 0;
  printf("f=%f   normalize_mirror_rad(f)=%f   expected=%f\n", f, fnm, expd);


  f = 2 * M_PI + 1;
  fnm = normalize_mirror_rad(f);
  expd = 1;
  printf("f=%f   normalize_mirror_rad(f)=%f   expected=%f\n", f, fnm, expd);


  f = - 2 * M_PI - 1.4;
  fnm = normalize_mirror_rad(f);
  expd = -1.4;
  printf("f=%f   normalize_mirror_rad(f)=%f   expected=%f\n", f, fnm, expd);


  f = - 2 * M_PI - 2.9;
  fnm = normalize_mirror_rad(f);
  expd = -2.9;
  printf("f=%f   normalize_mirror_rad(f)=%f   expected=%f\n", f, fnm, expd);


  f = - 3 * M_PI - 1;
  fnm = normalize_mirror_rad(f);
  expd = f + 4 * M_PI;
  printf("f=%f   normalize_mirror_rad(f)=%f   expected=%f\n", f, fnm, expd);


  f = -M_PI;
  float fnr = normalize_rad(f);
  expd = -M_PI;
  printf("f=%f   normalize_rad(f)=%f   expected=%f\n", f, fnr, expd);

  f = 3 * M_PI;
  fnr = normalize_rad(f);
  expd = M_PI;
  printf("f=%f   normalize_rad(f)=%f   expected=%f\n", f, fnr, expd);

  f = - 3 * M_PI;
  fnr = normalize_rad(f);
  expd = -M_PI;
  printf("f=%f   normalize_rad(f)=%f   expected=%f\n", f, fnr, expd);

  f = - 2 * M_PI - 1;
  fnr = normalize_rad(f);
  expd = -1;
  printf("f=%f   normalize_rad(f)=%f   expected=%f\n", f, fnr, expd);

  return 0;
}
