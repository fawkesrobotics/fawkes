#ifndef _COLLI_UTILS_OCCUPANCYGRID_PROBABILITY_H_
#define _COLLI_UTILS_OCCUPANCYGRID_PROBABILITY_H_

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

typedef float Probability;

inline bool
isProb(Probability p)
{
  return ((p >= 0) && (p <= 1));
}

} // namespace fawkes

#endif
