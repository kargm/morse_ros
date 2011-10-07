/*
 * Utils.cpp
 *
 *  Created on: Aug 24, 2009
 *      Author: kruset
 */

#include "M3DW_Utils.h"
#include <stddef.h>
#include <sys/time.h>


namespace NHPPlayerDriver {

long getCurrentSecs()
{
  timeval timeVal;
  long result = gettimeofday(&timeVal, NULL);

  if (result != -1) {
    result = timeVal.tv_sec;
  } else {
      result = 0;
  }
  return result;
}

long getCurrentMillisecs()
{
  timeval timeVal;
  long result = gettimeofday(&timeVal, NULL);

  if (result != -1) {
    result = ((timeVal.tv_sec % 1000) * 1000) + (timeVal.tv_usec / 1000);
  } else {
      result = 0;
  }
  return result;
}


}
