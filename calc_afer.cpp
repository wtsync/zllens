#include "calc_afer.h"
#include "Arduino.h"

Calc_afer::Calc_afer(unsigned int min_, unsigned int max_)
{
  p_lap = NULL;
  p_f = NULL;
  indexer = 0;
  count = 0;
  last_pos = 0;
  min_tar = min_;
  max_tar = max_;
}

bool Calc_afer::reset(unsigned int f, unsigned char range, unsigned char gap, unsigned int* first_f)
{
  unsigned int distance, left, right;
  if ( (0 >= range) ||
       (0 >= gap)   ||
       ((range * gap * 2 + 1) >= 100)
    )
  {
    range = 30;
    gap   = 1;
  }
  
  distance = range * gap;
  left = f - distance;
  if (left <= min_tar)
  {
    left = min_tar;
  }
  left = f + distance;
  if (left >= max_tar)
  {
    left = max_tar;
  }

  last_pos = f;
  count = right - left + 1;
  indexer = 0;

  if (NULL == p_lap)
  {
    delete [] p_lap;
  }
  if (NULL == p_f)
  {
    delete [] p_f;
  }
  p_lap = new unsigned int  [count];
  p_f   = new unsigned int  [count];
  for (int i = 0; i < count; ++i)
  {
    p_f[i]   = left + i;
    p_lap[i] = 0;
  }

  (*first_f) = p_f[indexer];
}
// return true when (*f) is OK. 
// return false when full
bool Calc_afer::set_lap_get_next_f(unsigned int lap, unsigned int * f)
{
  if (indexer < count)
  {
    p_lap[indexer] = lap;
    indexer++;
    if (indexer < count)
    {
      (*f) = p_f[indexer];
      return true;
    }
  }
  return false;
}
bool Calc_afer::get_best_f(unsigned int* f, unsigned int* lap)
{
  (*lap) = 0;
  (*f) = last_pos;
  for (int i = 0; i < count; ++i)
  {
    if (p_lap[i] > (*lap))
    {
      (*f)   = p_f[i];
      (*lap) = p_lap[i];
    }
  }

  return 0 != (*lap);
}
